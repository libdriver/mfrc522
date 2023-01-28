[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.svg" width="400" height="150"/>
</div>

## LibDriver MFRC522

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mfrc522/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE) 

Der MFRC522 ist ein hochintegrierter Lese-/Schreib-IC für kontaktlose Kommunikation bei 13,56 MHz. Der MFRC522-Leser unterstützt ISO/IEC 14443 A/MIFARE und NTAG. Der interne Sender des MFRC522 kann eine Lese-/Schreibantenne ansteuern, die für die Kommunikation mit ISO/IEC 14443 A/MIFARE-Karten und Transpondern ohne zusätzliche aktive Schaltungen ausgelegt ist. Das Empfängermodul bietet eine robuste und effiziente Implementierung zum Demodulieren und Decodieren von Signalen von ISO/IEC 14443 A/MIFARE-kompatiblen Karten und Transpondern. Das digitale Modul verwaltet die vollständige ISO/IEC 14443 A-Framing- und Fehlererkennungsfunktion (Parität und CRC).

Libdriver MFRC522 ist der voll funktionsfähige Treiber von MFRC522, der von LibDriver gestartet wurde. Er bietet die Funktion der kontaktlosen Kommunikation, CRC-Berechnung, Zufallsgenerierung und so weiter. LibDriver ist MISRA-konform.

LibDriver MFRC522 bietet den vollen Funktionstreiber von MFRC522, enthält jedoch nicht MIFARE Classic, MIFARE Ultralight, NTAG21x und andere Treiber.

MIFARE Classic-Treiber finden Sie unter https://github.com/hepingood/mifare_classic

MIFARE Ultralight-Treiber finden Sie unter https://github.com/hepingood/mifare_ultralight

Das NTAG21x-Laufwerk kann auf https://github.com/hepingood/ntag21x verwiesen werden

### Inhaltsverzeichnis

  - [Anweisung](#Anweisung)
  - [Installieren](#Installieren)
  - [Nutzung](#Nutzung)
    - [example basic](#example-basic)
  - [Dokument](#Dokument)
  - [Beitrag](#Beitrag)
  - [Lizenz](#Lizenz)
  - [Kontaktieren Sie uns](#Kontaktieren-Sie-uns)

### Anweisung

/src enthält LibDriver MFRC522-Quelldateien.

/interface enthält die plattformunabhängige Vorlage LibDriver MFRC522 IIC, SPI, UART.

/test enthält den Testcode des LibDriver MFRC522-Treibers und dieser Code kann die erforderliche Funktion des Chips einfach testen.

/example enthält LibDriver MFRC522-Beispielcode.

/doc enthält das LibDriver MFRC522-Offlinedokument.

/Datenblatt enthält MFRC522-Datenblatt.

/project enthält den allgemeinen Beispielcode für Linux- und MCU-Entwicklungsboards. Alle Projekte verwenden das Shell-Skript, um den Treiber zu debuggen, und die detaillierten Anweisungen finden Sie in der README.md jedes Projekts.

### Installieren

Verweisen Sie auf eine plattformunabhängige IIC, SPI, UART-Schnittstellenvorlage und stellen Sie Ihren Plattform-IIC, SPI, UART-Treiber fertig.

Fügen Sie /src, /interface und /example zu Ihrem Projekt hinzu.

### Nutzung

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

### Dokument

Online-Dokumente: [https://www.libdriver.com/docs/mfrc522/index.html](https://www.libdriver.com/docs/mfrc522/index.html).

Offline-Dokumente: /doc/html/index.html.

### Beitrag

Bitte beachten Sie CONTRIBUTING.md.

### Lizenz

Urheberrechte © (c) 2015 - Gegenwart LibDriver Alle Rechte vorbehalten



Die MIT-Lizenz (MIT)



Hiermit wird jeder Person kostenlos die Erlaubnis erteilt, eine Kopie zu erhalten

dieser Software und zugehörigen Dokumentationsdateien (die „Software“) zu behandeln

in der Software ohne Einschränkung, einschließlich, aber nicht beschränkt auf die Rechte

zu verwenden, zu kopieren, zu modifizieren, zusammenzuführen, zu veröffentlichen, zu verteilen, unterzulizenzieren und/oder zu verkaufen

Kopien der Software und Personen, denen die Software gehört, zu gestatten

dazu eingerichtet werden, unter folgenden Bedingungen:



Der obige Urheberrechtshinweis und dieser Genehmigungshinweis müssen in allen enthalten sein

Kopien oder wesentliche Teile der Software.



DIE SOFTWARE WIRD "WIE BESEHEN" BEREITGESTELLT, OHNE JEGLICHE GEWÄHRLEISTUNG, AUSDRÜCKLICH ODER

STILLSCHWEIGEND, EINSCHLIESSLICH, ABER NICHT BESCHRÄNKT AUF DIE GEWÄHRLEISTUNG DER MARKTGÄNGIGKEIT,

EIGNUNG FÜR EINEN BESTIMMTEN ZWECK UND NICHTVERLETZUNG VON RECHTEN DRITTER. IN KEINEM FALL DARF DAS

AUTOREN ODER URHEBERRECHTSINHABER HAFTEN FÜR JEGLICHE ANSPRÜCHE, SCHÄDEN ODER ANDERE

HAFTUNG, OB AUS VERTRAG, DELIKT ODER ANDERWEITIG, ENTSTEHEND AUS,

AUS ODER IM ZUSAMMENHANG MIT DER SOFTWARE ODER DER VERWENDUNG ODER ANDEREN HANDLUNGEN MIT DER

SOFTWARE.

### Kontaktieren Sie uns

Bitte senden Sie eine E-Mail an lishifenging@outlook.com.