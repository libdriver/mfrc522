[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MFRC522

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mfrc522/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MFRC522是一種高度集成的讀寫器IC，用於13.56 MHz的非接觸式通信。 MFRC522讀卡器支持ISO/IEC 14443 A/MIFARE和NTAG。 MFRC522的內部發射機能够驅動讀寫器天線，該天線設計用於與ISO/IEC 14443 A/MIFARE卡和轉發器的通信且無需額外的有源電路。 接收器模塊為解調和解碼來自ISO/IEC 14443 A/MIFARE相容卡和轉發器的訊號提供了一種穩健而高效的實現。 數位模塊管理完整的ISO/IEC 14443 A成幀和錯誤檢測（同位和CRC）功能。
LibDriver MFRC522是LibDriver推出的MFRC522全功能驅動，該驅動提供非接觸通信、CRC計算、亂數生成等功能並且它符合MISRA標準。

LibDriver MFRC522提供MFRC522的全功能驅動，但不包括MIFARE Classic，MIFARE Ultralight和NTAG21x等驅動。

MIFARE Classic驅動可參攷 https://github.com/libdriver/mifare_classic。

MIFARE Ultralight驅動可參攷 https://github.com/libdriver/mifare_ultralight。

NTAG21x驅動可參攷 https://github.com/libdriver/ntag21x。

### 目錄

  - [說明](#說明)
  - [安裝](#安裝)
  - [使用](#使用)
    - [example basic](#example-basic)
  - [文檔](#文檔)
  - [貢獻](#貢獻)
  - [版權](#版權)
  - [聯繫我們](#聯繫我們)

### 說明

/src目錄包含了LibDriver MFRC522的源文件。

/interface目錄包含了LibDriver MFRC522與平台無關的IIC, SPI, UART總線模板。

/test目錄包含了LibDriver MFRC522驅動測試程序，該程序可以簡單的測試芯片必要功能。

/example目錄包含了LibDriver MFRC522編程範例。

/doc目錄包含了LibDriver MFRC522離線文檔。

/datasheet目錄包含了MFRC522數據手冊。

/project目錄包含了常用Linux與單片機開發板的工程樣例。所有工程均採用shell腳本作為調試方法，詳細內容可參考每個工程裡面的README.md。

/misra目錄包含了LibDriver MISRA程式碼掃描結果。

### 安裝

參考/interface目錄下與平台無關的IIC, SPI, UART總線模板，完成指定平台的IIC, SPI, UART總線驅動。

將/src目錄，/interface目錄和/example目錄加入工程。

### 使用

#### example basic

```C
#include "driver_mfrc522_basic.h"

uint8_t i;
uint8_t buf[25];
uint16_t crc;
char crc_input[] = "libdriver"; 
uint8_t in_buf[64];
uint8_t in_len;
uint8_t out_len;
uint8_t out_buf[64];

static void a_callback(uint16_t type)
{
    switch (type)
    {
        case MFRC522_INTERRUPT_MFIN_ACT :
        {
            mfrc522_interface_debug_print("mfrc522: irq mfin act.\n");
            
            break;
        }
        case MFRC522_INTERRUPT_CRC :
        {
            break;
        }
        case MFRC522_INTERRUPT_TX :
        {
            break;
        }
        case MFRC522_INTERRUPT_RX :
        {
            break;
        }
        case MFRC522_INTERRUPT_IDLE :
        {
            break;
        }
        case MFRC522_INTERRUPT_HI_ALERT :
        {
            break;
        }
        case MFRC522_INTERRUPT_LO_ALERT :
        {
            break;
        }
        case MFRC522_INTERRUPT_ERR :
        {
            mfrc522_interface_debug_print("mfrc522: irq err.\n");
            
            break;
        }
        case MFRC522_INTERRUPT_TIMER :
        {
            mfrc522_interface_debug_print("mfrc522: irq timer.\n");
            
            break;
        }
        default :
        {
            mfrc522_interface_debug_print("mfrc522: irq unknown code.\n");
            
            break;
        }
    }
}

...
    
/* interrupt init */
g_gpio_irq = mfrc522_interrupt_irq_handler;
res = gpio_interrupt_init();
if (res != 0)
{
    return 1;
}

/* init */
res = mfrc522_basic_init(MFRC522_INTERFACE_SPI, 0x00, a_callback);
if (res != 0)
{
    (void)gpio_interrupt_deinit();
    g_gpio_irq = NULL;

    return 1;
}

...
    
/* get the random */
res = mfrc522_basic_generate_random(buf);
if (res != 0)
{
    (void)gpio_interrupt_deinit();
    g_gpio_irq = NULL;

    return 1;
}
for (i = 0; i < 25; i++)
{
    mfrc522_interface_debug_print("0x%02X ", buf[i]);
}
mfrc522_interface_debug_print("\n");

...
    
/* calculate the crc */
res = mfrc522_basic_calculate_crc((uint8_t *)crc_input, (uint8_t)strlen(crc_input), &crc);
if (res != 0)
{
    (void)gpio_interrupt_deinit();
    g_gpio_irq = NULL;

    return 1;
}
mfrc522_interface_debug_print("%s crc is 0x%04X.\n", argv[4], crc);

...

/* set the input */
in_buf[0] = 0x26;   
in_len = 1;
out_len = 64;
res = mfrc522_basic_transceiver(in_buf, in_len, out_buf, &out_len);
if (res != 0)
{
    (void)gpio_interrupt_deinit();
    g_gpio_irq = NULL;

    return 1;
}
for (i = 0; i < 25; i++)
{
    mfrc522_interface_debug_print("0x%02X ", out_buf[i]);
}
mfrc522_interface_debug_print("\n");

...
    
/* deinit */
(void)mfrc522_basic_deinit();
(void)gpio_interrupt_deinit();
g_gpio_irq = NULL;

return 0;
```

### 文檔

在線文檔: [https://www.libdriver.com/docs/mfrc522/index.html](https://www.libdriver.com/docs/mfrc522/index.html)。

離線文檔: /doc/html/index.html。

### 貢獻

請參攷CONTRIBUTING.md。

### 版權

版權 (c) 2015 - 現在 LibDriver 版權所有

MIT 許可證（MIT）

特此免費授予任何獲得本軟件副本和相關文檔文件（下稱“軟件”）的人不受限制地處置該軟件的權利，包括不受限制地使用、複製、修改、合併、發布、分發、轉授許可和/或出售該軟件副本，以及再授權被配發了本軟件的人如上的權利，須在下列條件下：

上述版權聲明和本許可聲明應包含在該軟件的所有副本或實質成分中。

本軟件是“如此”提供的，沒有任何形式的明示或暗示的保證，包括但不限於對適銷性、特定用途的適用性和不侵權的保證。在任何情況下，作者或版權持有人都不對任何索賠、損害或其他責任負責，無論這些追責來自合同、侵權或其它行為中，還是產生於、源於或有關於本軟件以及本軟件的使用或其它處置。

### 聯繫我們

請聯繫lishifenging@outlook.com。
