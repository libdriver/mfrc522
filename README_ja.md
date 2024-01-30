[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MFRC522

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mfrc522/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MFRC522は、13.56MHzでの非接触通信用の高度に統合されたリーダー/ライターICです。 MFRC522リーダーはISO/IEC 14443 A / MIFAREおよびNTAGをサポートします。MFRC522の内部送信機は、追加のアクティブ回路なしでISO / IEC 14443 A/MIFAREカードおよびトランスポンダーと通信するように設計されたリーダー/ライターアンテナを駆動できます。 受信機モジュールは、ISO / IEC 14443 A/MIFARE互換のカードおよびトランスポンダからの信号を復調およびデコードするための堅牢で効率的な実装を提供します。 デジタルモジュールは、完全なISO / IEC 14443 Aフレーミングおよびエラー検出（パリティおよびCRC）機能を管理します。

Libdriver MFRC522は、LibDriverによって起動されたMFRC522の全機能ドライバーであり、非接触通信、crc計算、ランダム生成などの機能を提供します。 LibDriverはMISRAに準拠しています。

LibDriver MFRC522は、MFRC522の全機能ドライバーを提供しますが、MIFARE Classic、MIFARE Ultralight、NTAG21xおよびその他のドライバーは含まれていません。

MIFARE Classicドライバーは、[https://github.com/libdriver/mifare_classic](https://github.com/libdriver/mifare_classic) を参照できます。

MIFARE Ultralightドライバーは、[https://github.com/libdriver/mifare_ultralight](https://github.com/libdriver/mifare_ultralight) を参照できます。

NTAG21x ドライバーは、[https://github.com/libdriver/ntag21x](https://github.com/libdriver/ntag21x) を参照できます。

### 目次

  - [説明](#説明)
  - [インストール](#インストール)
  - [使用](#使用)
    - [example basic](#example-basic)
  - [ドキュメント](#ドキュメント)
  - [貢献](#貢献)
  - [著作権](#著作権)
  - [連絡して](#連絡して)

### 説明

/ srcディレクトリには、LibDriver MFRC522のソースファイルが含まれています。

/ interfaceディレクトリには、LibDriver MFRC522用のプラットフォームに依存しないIIC, SPI, UARTバステンプレートが含まれています。

/ testディレクトリには、チップの必要な機能を簡単にテストできるLibDriver MFRC522ドライバーテストプログラムが含まれています。

/ exampleディレクトリには、LibDriver MFRC522プログラミング例が含まれています。

/ docディレクトリには、LibDriver MFRC522オフラインドキュメントが含まれています。

/ datasheetディレクトリには、MFRC522データシートが含まれています。

/ projectディレクトリには、一般的に使用されるLinuxおよびマイクロコントローラー開発ボードのプロジェクトサンプルが含まれています。 すべてのプロジェクトは、デバッグ方法としてシェルスクリプトを使用しています。詳細については、各プロジェクトのREADME.mdを参照してください。

/ misraはLibDriver misraコードスキャン結果を含む。

### インストール

/ interfaceディレクトリにあるプラットフォームに依存しないIIC, SPI, UARTバステンプレートを参照して、指定したプラットフォームのIIC, SPI, UARTバスドライバを完成させます。

/ srcディレクトリ、/ interfaceディレクトリ、および/exampleディレクトリをプロジェクトに追加します。

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

### ドキュメント

オンラインドキュメント: [https://www.libdriver.com/docs/mfrc522/index.html](https://www.libdriver.com/docs/mfrc522/index.html)。

オフラインドキュメント: /doc/html/index.html。

### 貢献

CONTRIBUTING.mdを参照してください。

### 著作権

著作権（c）2015-今 LibDriver 全著作権所有

MITライセンス（MIT）

このソフトウェアおよび関連するドキュメントファイル（「ソフトウェア」）のコピーを取得した人は、無制限の使用、複製、変更、組み込み、公開、配布、サブライセンスを含む、ソフトウェアを処分する権利を制限なく付与されます。ソフトウェアのライセンスおよび/またはコピーの販売、および上記のようにソフトウェアが配布された人の権利のサブライセンスは、次の条件に従うものとします。

上記の著作権表示およびこの許可通知は、このソフトウェアのすべてのコピーまたは実体に含まれるものとします。

このソフトウェアは「現状有姿」で提供され、商品性、特定目的への適合性、および非侵害の保証を含むがこれらに限定されない、明示または黙示を問わず、いかなる種類の保証もありません。 いかなる場合も、作者または著作権所有者は、契約、不法行為、またはその他の方法で、本ソフトウェアおよび本ソフトウェアの使用またはその他の廃棄に起因または関連して、請求、損害、またはその他の責任を負わないものとします。

### 連絡して

お問い合わせくださいlishifenging@outlook.com。