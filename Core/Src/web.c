 /*
 * web.c
 *
 *  Created on: Jun 8, 2021
 *      Author: LBekel
 */

#include "main.h"
#include "web.h"
#include "lwip/apps/httpd.h"
#include "string.h"
#include <stdio.h>
#include <stdbool.h>
#include "flash_if.h"
#include "jumpcode.h"

static __IO uint32_t DataFlag=0;
static __IO uint32_t size =0;
static __IO uint32_t FlashWriteAddress;
static uint32_t TotalReceived=0;
static char LeftBytesTab[4];
static uint8_t LeftBytes=0;
static __IO uint8_t resetpage=0;
static __IO uint32_t TotalData=0;
struct http_state
{
  char *file;
  u32_t left;
};

typedef enum
{
  FileUploadPage,
  UploadDonePage,
  ResetDonePage
}htmlpageState;

uint32_t alreadyLoggedIn = 0;
htmlpageState htmlpage;

static const char http_crnl_2[4] =
/* "\r\n--" */
{0xd, 0xa,0x2d,0x2d};
static const char octet_stream[14] =
/* "octet-stream" */
{0x6f, 0x63, 0x74, 0x65, 0x74, 0x2d, 0x73, 0x74, 0x72, 0x65, 0x61, 0x6d, 0x0d, };

#define USER_PASS_BUFSIZE 16

static void *current_connection;
static void *valid_connection;

static void IAP_HTTP_writedata(char* data, uint32_t len);

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;


err_t httpd_post_begin(void *connection, const char *uri, const char *http_request, u16_t http_request_len,
        int content_len, char *response_uri, u16_t response_uri_len, u8_t *post_auto_wnd)
{
    LWIP_UNUSED_ARG(connection);
    LWIP_UNUSED_ARG(http_request);
    LWIP_UNUSED_ARG(http_request_len);
    LWIP_UNUSED_ARG(content_len);
    LWIP_UNUSED_ARG(post_auto_wnd);
    if(!memcmp(uri, "/upload.cgi", 12))
    {
        htmlpage = FileUploadPage;
        if(current_connection != connection)
        {
            current_connection = connection;
            valid_connection = NULL;
            snprintf(response_uri, response_uri_len, "/upload.html");
            //*post_auto_wnd = 0;
            size = content_len;
            printf("Content Length: %ld\r\n",size);
            printf("FileUploadPage Begin\r\n");
            return ERR_OK;
        }
    }
    else if(!memcmp(uri, "/startapp.cgi", 14))
    {
        htmlpage = ResetDonePage;
        if(current_connection != connection)
        {
            current_connection = connection;
            valid_connection = NULL;
            snprintf(response_uri, response_uri_len, "/index.html");
            //*post_auto_wnd = 0;
            return ERR_OK;
        }
    }
    return ERR_VAL;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p)
{

    if(current_connection == connection)
    {
        if(htmlpage == FileUploadPage)
        {
            printf("FileUploadPage Data\r\n");
            char *data;
            char *ptr;
            uint32_t i = 0;
            uint32_t DataOffset = 0;
            uint32_t FilenameOffset;
            char filename[35];

            uint32_t len = p->tot_len;
            data = p->payload;

            /* POST Packet received */
            if(DataFlag == 0)
            {
                printf("POST Packet\r\n");
                TotalReceived = 0;
                /* parse packet for the octet-stream field */
                for(i = 0; i < len; i++)
                {
                    if(strncmp((char*) (data + i), octet_stream, 13) == 0)
                    {
                        DataOffset = i + 16;
                        break;
                    }
                }
                // no octet-stream data, this is no bin file
                if(DataOffset == 0)
                {
                    pbuf_free(p);
                    p = NULL;
                    return ERR_OK;
                }
                else
                {
                    TotalReceived = len - DataOffset;
                    size -= DataOffset;
                }
                DataFlag++;
                /* parse packet for the filename field */
                FilenameOffset = 0;
                for(i = 0; i < len; i++)
                {
                    if(strncmp((char*) (data + i), "filename=", 9) == 0)
                    {
                        FilenameOffset = i + 10;
                        break;
                    }
                }
                i = 0;
                if(FilenameOffset)
                {
                    while((*(data + FilenameOffset + i) != 0x22) && (i < 35))
                    {
                        filename[i] = *(data + FilenameOffset + i);
                        i++;
                    }
                    filename[i] = 0x0;
                    printf("Filename: %s\r\n", filename);
                }

                if(i == 0)
                {
                    htmlpage = FileUploadPage;
                    /* no filename, in this case reload upload page */
                    DataFlag = 0;
                    pbuf_free(p);
                    p = NULL;
                    return ERR_OK;
                }
                TotalData = 0;
                /* init flash */
                //FLASH_If_Init();
                if(HAL_FLASH_Unlock()!=HAL_OK){
                    printf("HAL_FLASH_Unlock failed\r\n");;
                }
                /* erase user flash area */
                if(FLASH_If_Erase(USER_FLASH_FIRST_PAGE_ADDRESS))
                {
                    printf("Flash Erase failed\r\n");
                }
                FlashWriteAddress = USER_FLASH_FIRST_PAGE_ADDRESS;
            }
            /* DataFlag >1 => the packet is data only  */
            else
            {
                printf("Data Packet\r\n");
                TotalReceived += len;
            }
            printf("TotalReceived %ld\r\n", TotalReceived);
            ptr = (char*) (data + DataOffset);
            len -= DataOffset;

            /* update Total data received counter */
            TotalData += len;
            printf("TotalData %ld\r\n", TotalData);
            /* check if last data packet */
            if(TotalReceived >= size)
            {
                printf("Data Complete\r\n");
                /* if last packet need to remove the http boundary tag */
                /* parse packet for "\r\n--" starting from end of data */
                i = 4;
                while(strncmp((char*) (data + p->tot_len - i), http_crnl_2, 4))
                {
                    i++;
                }
                len -= i;
                TotalData -= i;

                /* write data in Flash */
                if(len)
                {
                    IAP_HTTP_writedata(ptr, len);
                }
                if(HAL_FLASH_Lock()!=HAL_OK)
                {
                    printf("HAL_FLASH_Lock failed\r\n");;
                }
                DataFlag = 0;
                htmlpage = UploadDonePage;
                /* send uploaddone.html page */
            }
            /* not last data packet */
            else
            {
                printf("not Complete\r\n");
                /* write data in flash */
                if(len)
                {
                    IAP_HTTP_writedata(ptr, len);
                }
            }
            pbuf_free(p);
            p = NULL;
            return ERR_OK;
        }
        else if(htmlpage == ResetDonePage)
        {
            pbuf_free(p);
            p = NULL;
            return ERR_OK;
        }

    }
    return ERR_VAL;
}


void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
    /* default page is "login failed" */
    snprintf(response_uri, response_uri_len, "/loginfail.html");
    if(current_connection == connection)
    {
        if(htmlpage == FileUploadPage)
        {
            snprintf(response_uri, response_uri_len, "/upload.html");
            printf("FileUploadPage Finished\r\n");
        }
        else if(htmlpage == UploadDonePage)
        {
            snprintf(response_uri, response_uri_len, "/uploaddone.html");
            printf("UploadDonePage Finished\r\n");
        }
        else if(htmlpage == ResetDonePage)
        {
            /* Jump to user application */
            snprintf(response_uri, response_uri_len, "/startapp.html");
            setJumpCode(AUTO, 0);
            setReset();
            printf("Reset\r\n");
        }
    }
}


/**
  * @brief  writes received data in flash
  * @param  ptr: data pointer
  * @param  len: data length
  * @retval None
  */
static void IAP_HTTP_writedata(char *ptr, uint32_t len)
{
    uint32_t count, i = 0, j = 0;

    /* check if any left bytes from previous packet transfer*/
    /* if it is the case do a concat with new data to create a 32-bit word */
    if(LeftBytes)
    {
        while(LeftBytes <= 3)
        {
            if(len > (j + 1))
            {
                LeftBytesTab[LeftBytes++] = *(ptr + j);
            }
            else
            {
                LeftBytesTab[LeftBytes++] = 0xFF;
            }
            j++;
        }
        FLASH_If_Write(&FlashWriteAddress, (uint32_t*) (LeftBytesTab), 1);
        LeftBytes = 0;

        /* update data pointer */
        ptr = (char*) (ptr + j);
        len = len - j;
    }

    /* write received bytes into flash */
    count = len / 4;

    /* check if remaining bytes < 4 */
    i = len % 4;
    if(i > 0)
    {
        if(TotalReceived != size)
        {
            /* store bytes in LeftBytesTab */
            LeftBytes = 0;
            for(; i > 0; i--)
                LeftBytesTab[LeftBytes++] = *(char*) (ptr + len - i);
        }
        else
            count++;
    }
    FLASH_If_Write(&FlashWriteAddress, (uint32_t*) ptr, count);
}
