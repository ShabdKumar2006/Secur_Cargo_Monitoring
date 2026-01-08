/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_Port GPIOB

#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_Port GPIOB

#define LCD_D4_Pin GPIO_PIN_2
#define LCD_D4_Port GPIOB

#define LCD_D5_Pin GPIO_PIN_10
#define LCD_D5_Port GPIOB

#define LCD_D6_Pin GPIO_PIN_12
#define LCD_D6_Port GPIOB

static volatile uint32_t vibration_message_until_ms = 0;
static volatile uint32_t dht_display_until_ms = 0;

#define LCD_D7_Pin GPIO_PIN_13
#define LCD_D7_Port GPIOB

#define DHT22_Pin GPIO_PIN_0
#define DHT22_Port GPIOA

#define REED_Pin GPIO_PIN_3
#define REED_Port GPIOA

#define REED_ACTIVE_LOW 0u

#define SW420_Pin GPIO_PIN_4
#define SW420_Port GPIOA

#define SW420_ACTIVE_LOW 1u
#define SW420_POLL_MS 2u
#define SW420_DEBOUNCE_MS 2u
#define SW420_REARM_MS 800u

#define LDR_LIGHT_THRESHOLD 2000u
#define LDR_ACTIVE_LOW 1u
#define TEMP_RISE_THRESHOLD_C 25.0f

#define WIFI_SSID "Wifi_SSID"
#define WIFI_PASS "XYZ"

#define BRIDGE_HOST "*********"
#define BRIDGE_PORT 8080
#define BRIDGE_PATH "/ingest"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void LCD_Command(uint8_t cmd);
void LCD_String(char *str);
void LCD_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t cloud_enabled = 0;
static uint8_t firebase_last_err = 0;
static char esp_last_conn_resp[17] = "";
static char esp_last_http_resp[17] = "";

static volatile uint8_t sw420_irq_pending = 0;
static volatile uint32_t sw420_irq_last_ms = 0;
static volatile uint8_t sw420_force_refresh = 0;

static char cloud_lcd1[17] = "";
static char cloud_lcd2[17] = "";
static uint8_t cloud_msg_id = 0;
static uint8_t cloud_reed_open = 0;
static uint8_t cloud_reed_closed = 0;
static uint8_t cloud_light_detected = 0;
static uint8_t cloud_temp_rise = 0;
static uint8_t cloud_vibration = 0;
static float cloud_temp_c = 0.0f;
static float cloud_rh = 0.0f;
static uint8_t cloud_dht_ok = 0;
static uint16_t cloud_ldr_raw = 0;
static volatile uint8_t cloud_dirty = 0;
static volatile uint8_t cloud_force_send = 0;

static void Json_Escape(const char *in, char *out, size_t out_len)
{
  size_t o = 0;
  if ((out == NULL) || (out_len == 0))
  {
    return;
  }
  if (in == NULL)
  {
    out[0] = '\0';
    return;
  }
  for (size_t i = 0; in[i] != '\0'; i++)
  {
    char c = in[i];
    if (o >= (out_len - 1))
    {
      break;
    }
    if ((c == '\\') || (c == '"'))
    {
      if (o >= (out_len - 2)) break;
      out[o++] = '\\';
      out[o++] = c;
    }
    else if (c == '\n')
    {
      if (o >= (out_len - 2)) break;
      out[o++] = '\\';
      out[o++] = 'n';
    }
    else if (c == '\r')
    {
      if (o >= (out_len - 2)) break;
      out[o++] = '\\';
      out[o++] = 'r';
    }
    else if (c == '\t')
    {
      if (o >= (out_len - 2)) break;
      out[o++] = '\\';
      out[o++] = 't';
    }
    else if ((unsigned char)c < 0x20)
    {
      continue;
    }
    else
    {
      out[o++] = c;
    }
  }
  out[o] = '\0';
}

 static void ESP8266_SendStr(const char *s)
 {
   (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 1000);
 }

 static void ESP8266_FlushRx(uint32_t duration_ms)
 {
   uint32_t start = HAL_GetTick();
   uint8_t ch = 0;
   while ((HAL_GetTick() - start) < duration_ms)
   {
     (void)HAL_UART_Receive(&huart1, &ch, 1, 10);
   }
 }
 
 static uint16_t ESP8266_ReadResp(char *buf, uint16_t buf_len, uint32_t timeout_ms)
 {
  uint32_t start = HAL_GetTick();
  uint32_t last_rx = start;
  uint16_t idx = 0;
  uint8_t ch = 0;
  uint32_t idle_break_ms = 120u;
  uint8_t long_op = 0u;
  if (timeout_ms >= 5000u)
  {
    idle_break_ms = 1500u;
    long_op = 1u;
  }
 
  if (buf_len == 0)
  {
    return 0;
  }
 
  buf[0] = '\0';
 
  while ((HAL_GetTick() - start) < timeout_ms)
  {
    if (idx >= (uint16_t)(buf_len - 1))
    {
      break;
    }
 
    if (HAL_UART_Receive(&huart1, &ch, 1, 20) == HAL_OK)
    {
      last_rx = HAL_GetTick();
      buf[idx++] = (char)ch;
      buf[idx] = '\0';

      if ((strstr(buf, "\r\nOK\r\n") != NULL) ||
          (strstr(buf, "\nOK\r\n") != NULL) ||
          (strstr(buf, "\r\nERROR\r\n") != NULL) ||
          (strstr(buf, "\r\nFAIL\r\n") != NULL) ||
          (strstr(buf, "WIFI GOT IP") != NULL) ||
          (strstr(buf, "SEND OK") != NULL) ||
          (strstr(buf, ">") != NULL) ||
          (!long_op && ((strstr(buf, "ready") != NULL) ||
                        (strstr(buf, "ALREADY CONNECTED") != NULL) ||
                        (strstr(buf, "\r\nCONNECT\r\n") != NULL) ||
                        (strstr(buf, "\nCONNECT\r\n") != NULL) ||
                        (strstr(buf, "CONNECT\r\n") != NULL) ||
                        (strstr(buf, "CLOSED") != NULL))))
      {
        break;
      }
    }
    else
    {
      if ((idx > 0u) && ((HAL_GetTick() - last_rx) >= idle_break_ms))
      {
        break;
      }
    }
  }
 
  return idx;
 }
 
 static uint8_t ESP8266_AT_Probe(char *resp, uint16_t resp_len)
{
  ESP8266_FlushRx(1500);

  for (uint8_t attempt = 0; attempt < 8; attempt++)
  {
    ESP8266_SendStr("AT\r\n");
    (void)ESP8266_ReadResp(resp, resp_len, 1200);
    if ((strstr(resp, "OK") != NULL) || (strstr(resp, "ready") != NULL))
    {
      return 1u;
    }
    HAL_Delay(250);
    ESP8266_FlushRx(150);
  }

  ESP8266_SendStr("AT+RST\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 6000);
  ESP8266_FlushRx(1000);
  for (uint8_t attempt = 0; attempt < 8; attempt++)
  {
    ESP8266_SendStr("AT\r\n");
    (void)ESP8266_ReadResp(resp, resp_len, 1200);
    if ((strstr(resp, "OK") != NULL) || (strstr(resp, "ready") != NULL))
    {
      return 1u;
    }
    HAL_Delay(250);
    ESP8266_FlushRx(150);
  }

  {
    uint32_t prev_baud = huart1.Init.BaudRate;
    huart1.Init.BaudRate = 9600;
    if (HAL_UART_Init(&huart1) == HAL_OK)
    {
      ESP8266_FlushRx(1500);
      for (uint8_t attempt = 0; attempt < 8; attempt++)
      {
        ESP8266_SendStr("AT\r\n");
        (void)ESP8266_ReadResp(resp, resp_len, 1200);
        if ((strstr(resp, "OK") != NULL) || (strstr(resp, "ready") != NULL))
        {
          return 1u;
        }
        HAL_Delay(250);
        ESP8266_FlushRx(150);
      }

      ESP8266_SendStr("AT+RST\r\n");
      (void)ESP8266_ReadResp(resp, resp_len, 6000);
      ESP8266_FlushRx(1000);
      for (uint8_t attempt = 0; attempt < 8; attempt++)
      {
        ESP8266_SendStr("AT\r\n");
        (void)ESP8266_ReadResp(resp, resp_len, 1200);
        if ((strstr(resp, "OK") != NULL) || (strstr(resp, "ready") != NULL))
        {
          return 1u;
        }
        HAL_Delay(250);
        ESP8266_FlushRx(150);
      }
    }
    huart1.Init.BaudRate = prev_baud;
    (void)HAL_UART_Init(&huart1);
  }

  return 0u;
}

static uint8_t ESP8266_JoinWiFi(const char *ssid, const char *pass, char *resp, uint16_t resp_len)
{
  char cmd[192];

  ESP8266_SendStr("ATE0\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 800);

  ESP8266_SendStr("AT+CWMODE=1\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 1200);

  (void)snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
  ESP8266_SendStr(cmd);
  (void)ESP8266_ReadResp(resp, resp_len, 20000);
  if (strstr(resp, "WIFI GOT IP") != NULL)
  {
    return 1u;
  }
  if (strstr(resp, "WIFI CONNECTED") != NULL)
  {
    return 1u;
  }
  if (strstr(resp, "OK") != NULL)
  {
    return 1u;
  }
  return 0u;
}

static uint8_t ESP8266_BridgePostStatus(const char *lcd1,
                                       const char *lcd2,
                                       uint8_t msg_id,
                                       uint8_t reed_open,
                                       uint8_t reed_closed,
                                       uint8_t light_detected,
                                       uint8_t temp_rise,
                                       uint8_t vibration,
                                       float temp_c,
                                       float rh,
                                       uint8_t dht_ok,
                                       uint16_t ldr_raw,
                                       char *resp,
                                       uint16_t resp_len)
{
  char json[768];
  char req[1024];
  char cmd[128];
  int temp_c_x10 = 0;
  int rh_x10 = 0;
  char lcd1_esc[64];
  char lcd2_esc[64];

  firebase_last_err = 0;
  esp_last_conn_resp[0] = '\0';
  esp_last_http_resp[0] = '\0';

  if (dht_ok)
  {
    temp_c_x10 = (int)(temp_c * 10.0f);
    rh_x10 = (int)(rh * 10.0f);
  }

  Json_Escape(lcd1, lcd1_esc, sizeof(lcd1_esc));
  Json_Escape(lcd2, lcd2_esc, sizeof(lcd2_esc));

  int json_n = snprintf(json, sizeof(json),
                 "{\"device\":\"stm32\","
                 "\"lcd\":{\"line1\":\"%s\",\"line2\":\"%s\",\"msg_id\":%u},"
                 "\"msg_id\":%u,"
                 "\"flags\":{\"reed_open\":%u,\"reed_closed\":%u,\"light_detected\":%u,\"temp_rise\":%u,\"vibration\":%u},"
                 "\"sensors\":{\"temp_c_x10\":%d,\"rh_x10\":%d,\"dht_ok\":%u,\"ldr_raw\":%u},"
                 "\"ts_ms\":%lu}",
                 lcd1_esc, lcd2_esc, (unsigned)msg_id,
                 (unsigned)msg_id,
                 (unsigned)reed_open, (unsigned)reed_closed, (unsigned)light_detected, (unsigned)temp_rise, (unsigned)vibration,
                 temp_c_x10, rh_x10, (unsigned)dht_ok, (unsigned)ldr_raw,
                 (unsigned long)HAL_GetTick());

  if ((json_n < 0) || (json_n >= (int)sizeof(json)))
  {
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "JSON TRUNC");
    firebase_last_err = 3;
    return 0u;
  }

  int req_n = snprintf(req, sizeof(req),
                 "POST %s HTTP/1.1\r\n"
                 "Host: %s:%u\r\n"
                 "User-Agent: stm32-esp8266\r\n"
                 "Connection: close\r\n"
                 "Content-Type: application/json\r\n"
                 "Content-Length: %u\r\n"
                 "\r\n"
                 "%s",
                 BRIDGE_PATH,
                 BRIDGE_HOST,
                 (unsigned)BRIDGE_PORT,
                 (unsigned)strlen(json),
                 json);

  if ((req_n < 0) || (req_n >= (int)sizeof(req)))
  {
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "REQ TRUNC");
    firebase_last_err = 3;
    return 0u;
  }

  ESP8266_SendStr("AT+CIPMUX=0\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 1200);

  ESP8266_SendStr("AT+CIPMODE=0\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 1200);

  ESP8266_SendStr("AT+CIPCLOSE\r\n");
  (void)ESP8266_ReadResp(resp, resp_len, 800);

  HAL_Delay(200);

  ESP8266_FlushRx(100);
  (void)snprintf(cmd,
                 sizeof(cmd),
                 "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                 BRIDGE_HOST,
                 (unsigned)BRIDGE_PORT);
  ESP8266_SendStr(cmd);
  (void)ESP8266_ReadResp(resp, resp_len, 12000);
  if ((strstr(resp, "ERROR") != NULL) || (strstr(resp, "busy") != NULL))
  {
    HAL_Delay(300);
    ESP8266_SendStr("AT+CIPCLOSE\r\n");
    (void)ESP8266_ReadResp(resp, resp_len, 800);
    HAL_Delay(200);
    ESP8266_SendStr(cmd);
    (void)ESP8266_ReadResp(resp, resp_len, 12000);
  }
  if ((strstr(resp, "CONNECT") == NULL) &&
      (strstr(resp, "OK") == NULL) &&
      (strstr(resp, "ALREADY CONNECTED") == NULL) &&
      (strstr(resp, "Linked") == NULL))
  {
    if ((resp == NULL) || (resp[0] == '\0'))
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "NO RESP");
    }
    else if (strstr(resp, "DNS Fail") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "DNS Fail");
    }
    else if (strstr(resp, "DNS fail") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "DNS fail");
    }
    else if (strstr(resp, "FAIL") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "FAIL");
    }
    else if (strstr(resp, "ERROR") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "ERROR");
    }
    else if (strstr(resp, "busy") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "busy");
    }
    else if (strstr(resp, "link is not") != NULL)
    {
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%s", "link invalid");
    }
    else
    {
      size_t rlen = strlen(resp);
      const char *tail = resp;
      if (rlen > 16u)
      {
        tail = &resp[rlen - 16u];
      }
      (void)snprintf(esp_last_conn_resp, sizeof(esp_last_conn_resp), "%.16s", tail);
    }
    firebase_last_err = 1;
    return 0u;
  }

  ESP8266_FlushRx(80);
  HAL_Delay(60);

  for (uint8_t send_try = 0u; send_try < 2u; send_try++)
  {
    (void)snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u\r\n", (unsigned)strlen(req));
    ESP8266_SendStr(cmd);
    (void)ESP8266_ReadResp(resp, resp_len, 6000);
    if ((strstr(resp, ">") == NULL) && (strstr(resp, "OK") != NULL))
    {
      char resp2[128];
      (void)ESP8266_ReadResp(resp2, (uint16_t)sizeof(resp2), 2000);
      if (strstr(resp2, ">") != NULL)
      {
        (void)snprintf(resp, resp_len, "%s%s", resp, resp2);
      }
    }
    if (strstr(resp, ">") != NULL)
    {
      break;
    }

    if (send_try == 0u)
    {
      ESP8266_SendStr("AT+CIPCLOSE\r\n");
      (void)ESP8266_ReadResp(resp, resp_len, 1200);
      HAL_Delay(200);
      ESP8266_FlushRx(100);
      (void)snprintf(cmd,
                     sizeof(cmd),
                     "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                     BRIDGE_HOST,
                     (unsigned)BRIDGE_PORT);
      ESP8266_SendStr(cmd);
      (void)ESP8266_ReadResp(resp, resp_len, 12000);
      HAL_Delay(120);
      ESP8266_FlushRx(80);
    }
  }

  if (strstr(resp, ">") == NULL)
  {
    firebase_last_err = 2;
    return 0u;
  }

  ESP8266_SendStr(req);
  (void)ESP8266_ReadResp(resp, resp_len, 12000);

  if ((resp == NULL) || (resp[0] == '\0'))
  {
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "NO RESP");
    firebase_last_err = 3;
    return 0u;
  }

  if (strstr(resp, "SEND OK") == NULL)
  {
    size_t rlen0 = strlen(resp);
    const char *tail0 = resp;
    if (rlen0 > 16u)
    {
      tail0 = &resp[rlen0 - 16u];
    }
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%.16s", tail0);
    firebase_last_err = 3;
    return 0u;
  }

  if ((strstr(resp, "HTTP/1.1 4") != NULL) ||
      (strstr(resp, "HTTP/1.0 4") != NULL) ||
      (strstr(resp, "HTTP/1.1 5") != NULL) ||
      (strstr(resp, "HTTP/1.0 5") != NULL) ||
      (strstr(resp, " 400 ") != NULL))
  {
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "HTTP ERR");
    firebase_last_err = 3;
    return 0u;
  }

  if ((strstr(resp, "HTTP/1.1 2") != NULL) ||
      (strstr(resp, "HTTP/1.0 2") != NULL) ||
      (strstr(resp, " 200 ") != NULL) ||
      (strstr(resp, " 201 ") != NULL) ||
      (strstr(resp, "200") != NULL))
  {
    return 1u;
  }

  ESP8266_ReadResp(resp, resp_len, 8000);
  if ((strstr(resp, "HTTP/1.1 4") != NULL) ||
      (strstr(resp, "HTTP/1.0 4") != NULL) ||
      (strstr(resp, "HTTP/1.1 5") != NULL) ||
      (strstr(resp, "HTTP/1.0 5") != NULL) ||
      (strstr(resp, " 400 ") != NULL))
  {
    (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "HTTP ERR");
    firebase_last_err = 3;
    return 0u;
  }
  if ((strstr(resp, "HTTP/1.1 2") != NULL) ||
      (strstr(resp, "HTTP/1.0 2") != NULL) ||
      (strstr(resp, " 200 ") != NULL) ||
      (strstr(resp, " 201 ") != NULL) ||
      (strstr(resp, "200") != NULL))
  {
    return 1u;
  }

  (void)snprintf(esp_last_http_resp, sizeof(esp_last_http_resp), "%s", "SEND OK");
  return 1u;
}

static void DWT_Delay_Init(void)
{
  if ((CoreDebug-> DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static void DHT22_SetAsOutputODPullup(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_Port, &GPIO_InitStruct);
}

static void DHT22_SetAsInputPullup(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_Port, &GPIO_InitStruct);
}

static uint8_t DHT22_WaitForPinState(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_ticks = timeout_us * (SystemCoreClock / 1000000u);
  while (HAL_GPIO_ReadPin(DHT22_Port, DHT22_Pin) != state)
  {
    if ((DWT->CYCCNT - start) > timeout_ticks)
    {
      return 0;
    }
  }
  return 1;
}

static uint8_t DHT22_Read(float *temp_c, float *rh)
{
  uint8_t data[5] = {0, 0, 0, 0, 0};

  DHT22_SetAsOutputODPullup();
  HAL_GPIO_WritePin(DHT22_Port, DHT22_Pin, GPIO_PIN_RESET);
  delay_us(1200);
  HAL_GPIO_WritePin(DHT22_Port, DHT22_Pin, GPIO_PIN_SET);
  delay_us(30);

  DHT22_SetAsInputPullup();

  if (!DHT22_WaitForPinState(GPIO_PIN_RESET, 100)) return 0;
  if (!DHT22_WaitForPinState(GPIO_PIN_SET, 120)) return 0;
  if (!DHT22_WaitForPinState(GPIO_PIN_RESET, 120)) return 0;

  for (uint8_t i = 0; i < 40; i++)
  {
    if (!DHT22_WaitForPinState(GPIO_PIN_SET, 80)) return 0;
    uint32_t start = DWT->CYCCNT;
    if (!DHT22_WaitForPinState(GPIO_PIN_RESET, 120)) return 0;
    uint32_t pulse_ticks = (DWT->CYCCNT - start);
    uint32_t pulse_us = pulse_ticks / (SystemCoreClock / 1000000u);

    uint8_t bit = (pulse_us > 50u) ? 1u : 0u;
    data[i / 8] <<= 1;
    data[i / 8] |= bit;
  }

  uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (checksum != data[4])
  {
    return 0;
  }

  uint16_t raw_rh = (uint16_t)((data[0] << 8) | data[1]);
  uint16_t raw_t = (uint16_t)((data[2] << 8) | data[3]);

  if (rh)
  {
    *rh = raw_rh / 10.0f;
  }
  if (temp_c)
  {
    if (raw_t & 0x8000u)
    {
      raw_t &= 0x7FFFu;
      *temp_c = -(raw_t / 10.0f);
    }
    else
    {
      *temp_c = raw_t / 10.0f;
    }
  }
  return 1;
}

static uint16_t LDR_ReadRaw(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return 0xFFFFu;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0xFFFFu;
  }
  uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return val;
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
  uint8_t addr = 0x00;
  if (row == 0)
  {
    addr = (uint8_t)(0x00 + col);
  }
  else
  {
    addr = (uint8_t)(0x40 + col);
  }
  LCD_Command((uint8_t)(0x80 | addr));
}

static void LCD_Clear(void)
{
  LCD_Command(0x01);
  HAL_Delay(2);
}

static void LCD_PrintTwoLines(const char *l1, const char *l2)
{
  LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_String((char *)l1);
  LCD_SetCursor(1, 0);
  LCD_String((char *)l2);
}

void LCD_Enable(void)
{
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send4Bits(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_Command(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    LCD_Send4Bits(cmd >> 4);
    LCD_Enable();
    LCD_Send4Bits(cmd & 0x0F);
    LCD_Enable();
    HAL_Delay(2);
}

void LCD_Data(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);
    LCD_Send4Bits(data >> 4);
    LCD_Enable();
    LCD_Send4Bits(data & 0x0F);
    LCD_Enable();
    HAL_Delay(1);
}
void LCD_Init(void)
{
    HAL_Delay(50);

    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_RESET);

    LCD_Send4Bits(0x03);
    LCD_Enable();
    HAL_Delay(5);
    LCD_Send4Bits(0x03);
    LCD_Enable();
    HAL_Delay(1);
    LCD_Send4Bits(0x03);
    LCD_Enable();
    HAL_Delay(1);

    LCD_Send4Bits(0x02);
    LCD_Enable();
    HAL_Delay(1);

    LCD_Command(0x28);
    LCD_Command(0x08);
    LCD_Command(0x01);
    HAL_Delay(2);
    LCD_Command(0x06);
    LCD_Command(0x0C);
}

void LCD_String(char *str)
{
    while (*str)
        LCD_Data(*str++);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  HAL_Delay(1000);
  LCD_Init();
  LCD_PrintTwoLines("Secure Cargo", "Monitoring");
  HAL_Delay(1000);
 
  {
     char esp_resp_startup[256];
     uint8_t ok = ESP8266_AT_Probe(esp_resp_startup, (uint16_t)sizeof(esp_resp_startup));
     if (ok)
     {
       LCD_PrintTwoLines("ESP8266 AT", "OK");
     }
     else
     {
       LCD_PrintTwoLines("ESP8266 AT", "FAIL");
     }
     HAL_Delay(1000);

     if (ok)
     {
       uint8_t wifi_ok = 0u;
       while (!wifi_ok)
       {
         LCD_PrintTwoLines("WiFi", "Connecting");
         wifi_ok = ESP8266_JoinWiFi(WIFI_SSID, WIFI_PASS, esp_resp_startup, (uint16_t)sizeof(esp_resp_startup));
         if (!wifi_ok)
         {
           LCD_PrintTwoLines("WiFi", "Retry...");
           HAL_Delay(2000);
         }
       }

       if (wifi_ok)
       {
         LCD_PrintTwoLines("WiFi", "OK");
         cloud_enabled = 1;

         {
          uint8_t fb_ok = 0u;
          uint32_t server_start_ms = HAL_GetTick();

          while (!fb_ok && ((HAL_GetTick() - server_start_ms) < 60000u))
          {
            LCD_PrintTwoLines("Server", "Connecting");
            fb_ok = ESP8266_BridgePostStatus("Boot",
                                            "Online",
                                            255,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0.0f,
                                            0.0f,
                                            0,
                                            0,
                                            esp_resp_startup,
                                            (uint16_t)sizeof(esp_resp_startup));
            if (!fb_ok)
            {
              HAL_Delay(1500);
            }
          }
          if (fb_ok)
          {
            LCD_PrintTwoLines("Server", "OK");
            cloud_enabled = 1;
          }
           else
           {
             if (firebase_last_err == 1)
             {
               LCD_PrintTwoLines("Server E1 CONN", esp_last_conn_resp);
             }
             else if (firebase_last_err == 2)
             {
               LCD_PrintTwoLines("Server", "E2 SEND");
             }
             else if (firebase_last_err == 3)
             {
               LCD_PrintTwoLines("Server E3 HTTP", esp_last_http_resp);
             }
             else
             {
               LCD_PrintTwoLines("Server", "FAIL");
             }
             cloud_enabled = 0;
           }
           HAL_Delay(1000);
         }
       }
       else
       {
         LCD_PrintTwoLines("WiFi", "FAIL");
         cloud_enabled = 0;
        }
        HAL_Delay(1000);
     }

     LCD_PrintTwoLines("Secure Cargo", "Monitoring");
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_poll_ms = 0;
    static uint32_t last_sw420_poll_ms = 0;
    static uint32_t last_reed_poll_ms = 0;
    static uint32_t last_dht_ms = 0;
    static uint8_t dht_first_read = 1;
    static float last_temp_c = 0.0f;
    static uint8_t last_temp_valid = 0;
    static float last_dht_temp_c = 0.0f;
    static float last_dht_rh = 0.0f;
    static uint8_t last_dht_ok = 0;
    static uint8_t last_msg_id = 0xFF;
    static uint8_t last_light_detected = 0xFF;
    static uint8_t last_reed_open = 0xFF;
    static uint32_t last_hb_ms = 0;
    static uint32_t last_cloud_send_ms = 0;

    char esp_resp[256];

    uint32_t now = HAL_GetTick();

    static uint8_t reed_open = 0;
    static uint8_t reed_closed = 0;
    static uint8_t light_detected = 0;
    static uint8_t temp_rise = 0;
    static uint8_t disturbance = 0;
    disturbance = (HAL_GetTick() < vibration_message_until_ms) ? 1u : 0u;

    static GPIO_PinState reed_state_raw_last = GPIO_PIN_RESET;
    static GPIO_PinState reed_state_stable = GPIO_PIN_RESET;
    static uint32_t reed_last_change_ms = 0;

    if ((now - last_reed_poll_ms) >= 10)
    {
      last_reed_poll_ms = now;

      GPIO_PinState reed_raw = HAL_GPIO_ReadPin(REED_Port, REED_Pin);

      if (reed_raw != reed_state_raw_last)
      {
        reed_state_raw_last = reed_raw;
        reed_last_change_ms = now;
      }

      if ((reed_state_stable != reed_state_raw_last) && ((now - reed_last_change_ms) >= 50))
      {
        reed_state_stable = reed_state_raw_last;
      }

      if (REED_ACTIVE_LOW)
      {
        reed_closed = (reed_state_stable == GPIO_PIN_RESET) ? 1u : 0u;
        reed_open = (reed_state_stable == GPIO_PIN_SET) ? 1u : 0u;
      }
      else
      {
        reed_closed = (reed_state_stable == GPIO_PIN_SET) ? 1u : 0u;
        reed_open = (reed_state_stable == GPIO_PIN_RESET) ? 1u : 0u;
      }
    }
    static uint32_t sw420_active_since_ms = 0;
    static uint32_t sw420_rearm_until_ms = 0;
    static uint8_t vibration_event_pending = 0;
    static uint8_t sw420_active_last = 0;

    if (sw420_force_refresh)
    {
      sw420_force_refresh = 0u;
      vibration_event_pending = 1u;
    }

    if (sw420_irq_pending)
    {
      sw420_irq_pending = 0u;
      if (now >= sw420_rearm_until_ms)
      {
        vibration_message_until_ms = now + 1000u;
        sw420_rearm_until_ms = now + (uint32_t)SW420_REARM_MS;
        sw420_active_since_ms = 0u;
        vibration_event_pending = 1u;
      }
    }

    if ((now - last_sw420_poll_ms) >= (uint32_t)SW420_POLL_MS)
    {
      last_sw420_poll_ms = now;

      GPIO_PinState sw420_state = HAL_GPIO_ReadPin(SW420_Port, SW420_Pin);
      uint8_t sw420_active = 0;
      if (SW420_ACTIVE_LOW)
      {
        sw420_active = (sw420_state == GPIO_PIN_RESET) ? 1u : 0u;
      }
      else
      {
        sw420_active = (sw420_state == GPIO_PIN_SET) ? 1u : 0u;
      }

      if (now < sw420_rearm_until_ms)
      {
        sw420_active_since_ms = 0;
      }
      else
      {
        if (sw420_active && !sw420_active_last)
        {
          vibration_message_until_ms = now + 1000u;
          sw420_rearm_until_ms = now + (uint32_t)SW420_REARM_MS;
          sw420_active_since_ms = 0u;
          vibration_event_pending = 1u;
        }

        if (sw420_active)
        {
          if (sw420_active_since_ms == 0u)
          {
            sw420_active_since_ms = now;
          }
          else if ((now - sw420_active_since_ms) >= (uint32_t)SW420_DEBOUNCE_MS)
          {
            vibration_message_until_ms = now + 1000u;
            sw420_rearm_until_ms = now + (uint32_t)SW420_REARM_MS;
            sw420_active_since_ms = 0u;
            vibration_event_pending = 1u;
          }
        }
        else
        {
          sw420_active_since_ms = 0u;
        }
      }

      sw420_active_last = sw420_active;
    }

    if ((now - last_poll_ms) >= 200)
    {
      last_poll_ms = now;

      uint16_t ldr = LDR_ReadRaw();
      if (ldr != 0xFFFFu)
      {
        uint16_t th_low = (LDR_LIGHT_THRESHOLD > 200u) ? (uint16_t)(LDR_LIGHT_THRESHOLD - 200u) : 0u;
        uint16_t th_high = (uint16_t)(LDR_LIGHT_THRESHOLD + 200u);

        if (LDR_ACTIVE_LOW)
        {
          if (!light_detected)
          {
            light_detected = (ldr < th_low) ? 1u : 0u;
          }
          else
          {
            light_detected = (ldr > th_high) ? 0u : 1u;
          }
        }
        else
        {
          if (!light_detected)
          {
            light_detected = (ldr > th_high) ? 1u : 0u;
          }
          else
          {
            light_detected = (ldr < th_low) ? 0u : 1u;
          }
        }
      }
    }

    if (dht_first_read || ((now - last_dht_ms) >= 10000))
    {
      last_dht_ms = now;
      dht_first_read = 0;
      temp_rise = 0;
      float t = 0.0f;
      float h = 0.0f;
      if (DHT22_Read(&t, &h))
      {
        last_dht_temp_c = t;
        last_dht_rh = h;
        last_dht_ok = 1;
        dht_display_until_ms = now + 1000;

        if (last_temp_valid)
        {
          float diff = t - last_temp_c;
          if (diff < 0) diff = -diff;
          if (diff > TEMP_RISE_THRESHOLD_C)
          {
            temp_rise = 1;
          }
        }
        last_temp_c = t;
        last_temp_valid = 1;
      }
      else
      {
        last_dht_ok = 0;
        dht_display_until_ms = now + 1000;
      }
    }
    
    uint8_t msg_id = 0;

    if (HAL_GetTick() < vibration_message_until_ms)
    {
      msg_id = 6;
    }
    else if (HAL_GetTick() < dht_display_until_ms)
    {
      msg_id = 7;
    }
    else if (disturbance)
    {
      msg_id = 4;
    }
    else if (temp_rise)
    {
      msg_id = 3;
    }
    else if (reed_open && light_detected)
    {
      msg_id = 2;
    }
    else if (reed_open || light_detected)
    {
      msg_id = 1;
    }
    else if (reed_closed)
    {
      msg_id = 5;
    }
    else
    {
      msg_id = 0;
    }

    if (vibration_event_pending)
    {
      last_msg_id = 0xFF;
      vibration_event_pending = 0u;
    }

    if ((msg_id == 1u) || (msg_id == 2u) || (msg_id == 5u) || (msg_id == 0u))
    {
      if ((last_light_detected != light_detected) || (last_reed_open != reed_open))
      {
        last_msg_id = 0xFF;
      }
    }

    if (msg_id != last_msg_id)
    {
      last_msg_id = msg_id;
      last_light_detected = light_detected;
      last_reed_open = reed_open;

      const char *lcd1 = "";
      const char *lcd2 = "";

      uint8_t vibration = (HAL_GetTick() < vibration_message_until_ms) ? 1u : 0u;
      uint16_t ldr_raw = LDR_ReadRaw();

      if (msg_id == 4)
      {
        lcd1 = "Disturbance";
        lcd2 = "detected";
      }
      else if (msg_id == 7)
      {
        if (last_dht_ok)
        {
          char line1[17];
          char line2[17];

          int t10 = (int)(last_dht_temp_c * 10.0f);
          int h10 = (int)(last_dht_rh * 10.0f);
          if (t10 < 0) t10 = -t10;
          if (h10 < 0) h10 = -h10;
          (void)snprintf(line1, sizeof(line1), "Temp:%d.%dC", t10 / 10, t10 % 10);
          (void)snprintf(line2, sizeof(line2), "Hum :%d.%d%%", h10 / 10, h10 % 10);
          LCD_PrintTwoLines(line1, line2);

          if (cloud_enabled)
          {
            uint8_t put_ok = ESP8266_BridgePostStatus(line1,
                                                      line2,
                                                      msg_id,
                                                      reed_open,
                                                      reed_closed,
                                                      light_detected,
                                                      temp_rise,
                                                      vibration,
                                                      last_dht_temp_c,
                                                      last_dht_rh,
                                                      last_dht_ok,
                                                      ldr_raw,
                                                      esp_resp,
                                                      (uint16_t)sizeof(esp_resp));
            if (!put_ok)
            {
              (void)put_ok;
            }
          }
          goto lcd_cloud_done;
        }
        else
        {
          lcd1 = "DHT22 Error";
          lcd2 = "Check wiring";
        }
      }
      else if (msg_id == 6)
      {
        lcd1 = "Vibration";
        lcd2 = "detected";
      }
      else if (msg_id == 3)
      {
        lcd1 = "Temp Rise";
        lcd2 = "";
      }
      else if (msg_id == 2)
      {
        lcd1 = "Cargo Opened";
        lcd2 = "Not Secure";
      }
      else if (msg_id == 1)
      {
        if (light_detected)
        {
          lcd1 = "Cargo Opened";
          lcd2 = "Not Secure";
        }
        else
        {
          lcd1 = "Cargo closed";
          lcd2 = "Secured";
        }
      }
      else if (msg_id == 5)
      {
        lcd1 = "Cargo secured";
        lcd2 = "";
      }
      else
      {
        lcd1 = "Secure Cargo";
        lcd2 = "Monitoring";
      }

      LCD_PrintTwoLines(lcd1, lcd2);

      (void)snprintf(cloud_lcd1, sizeof(cloud_lcd1), "%.16s", lcd1);
      (void)snprintf(cloud_lcd2, sizeof(cloud_lcd2), "%.16s", lcd2);
      cloud_msg_id = msg_id;
      cloud_reed_open = reed_open;
      cloud_reed_closed = reed_closed;
      cloud_light_detected = light_detected;
      cloud_temp_rise = temp_rise;
      cloud_vibration = vibration;
      cloud_temp_c = last_dht_temp_c;
      cloud_rh = last_dht_rh;
      cloud_dht_ok = last_dht_ok;
      cloud_ldr_raw = ldr_raw;
      cloud_dirty = 1u;
      cloud_force_send = 1u;

lcd_cloud_done:
      (void)ldr_raw;
    }

    if (cloud_enabled && cloud_dirty)
    {
      uint32_t send_interval_ms = 0u;
      uint8_t force = 0u;
      if (cloud_force_send)
      {
        force = 1u;
      }

      if (force || ((now - last_cloud_send_ms) >= send_interval_ms))
      {
        cloud_force_send = 0u;
        cloud_dirty = 0u;
        last_cloud_send_ms = now;

        uint8_t put_ok = ESP8266_BridgePostStatus(cloud_lcd1,
                                                  cloud_lcd2,
                                                  cloud_msg_id,
                                                  cloud_reed_open,
                                                  cloud_reed_closed,
                                                  cloud_light_detected,
                                                  cloud_temp_rise,
                                                  cloud_vibration,
                                                  cloud_temp_c,
                                                  cloud_rh,
                                                  cloud_dht_ok,
                                                  cloud_ldr_raw,
                                                  esp_resp,
                                                  (uint16_t)sizeof(esp_resp));
        if (!put_ok)
        {
          cloud_dirty = 1u;
        }
      }
    }

    if ((now - last_hb_ms) >= 500u)
    {
      last_hb_ms = now;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SW420_Pin)
  {
    uint32_t now = HAL_GetTick();
    if ((now - sw420_irq_last_ms) >= 5u)
    {
      sw420_irq_last_ms = now;
      sw420_irq_pending = 1u;
      vibration_message_until_ms = now + 1000u;
      sw420_force_refresh = 1u;
    }
    return;
  }

  (void)GPIO_Pin;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**1
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
