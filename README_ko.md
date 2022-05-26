[English](/README.md) | [ 简体中文](/README_zh-Hans.md) | [繁體中文](/README_zh-Hant.md) | [日本語](/README_ja.md) | [Deutsch](/README_de.md) | [한국어](/README_ko.md)

<div align=center>
<img src="/doc/image/logo.png"/>
</div>

## LibDriver MFRC522

[![MISRA](https://img.shields.io/badge/misra-compliant-brightgreen.svg)](/misra/README.md) [![API](https://img.shields.io/badge/api-reference-blue.svg)](https://www.libdriver.com/docs/mfrc522/index.html) [![License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](/LICENSE)

MFRC522는 13.56MHz에서 비접촉 통신을 위한 고집적 리더/라이터 IC입니다. MFRC522 리더는 ISO/IEC 14443 A/MIFARE 및 NTAG를 지원합니다. MFRC522의 내부 송신기는 추가 활성 회로 없이 ISO/IEC 14443 A/MIFARE 카드 및 트랜스폰더와 통신하도록 설계된 리더/라이터 안테나를 구동할 수 있습니다. 수신기 모듈은 ISO/IEC 14443 A/MIFARE 호환 카드 및 응답기의 신호를 복조 및 디코딩하기 위한 강력하고 효율적인 구현을 제공합니다. 디지털 모듈은 완전한 ISO/IEC 14443 A 프레이밍 및 오류 감지(패리티 및 CRC) 기능을 관리합니다.

Libdriver MFRC522는 LibDriver에서 출시한 MFRC522의 전체 기능 드라이버입니다. 비접촉 통신, crc 계산, 임의 생성 등의 기능을 제공합니다. LibDriver는 MISRA를 준수합니다.

LibDriver MFRC522는 MFRC522의 전체 기능 드라이버를 제공하지만 MIFARE Classic, MIFARE Ultralight, NTAG21x 및 기타 드라이버는 포함하지 않습니다.
MIFARE Classic 드라이버는 https://github.com/hepingood/mifare_classic을 참조할 수 있습니다.
MIFARE Ultralight 드라이버는 https://github.com/hepingood/mifare_ultralight를 참조할 수 있습니다.
NTAG21x 드라이브는 https://github.com/hepingood/ntag21x를 참조할 수 있습니다.

### 콘텐츠

  - [설명](#설명)
  - [설치](#설치)
  - [사용](#사용)
    - [example basic](#example-basic)
  - [문서](#문서)
  - [기고](#기고)
  - [저작권](#저작권)
  - [문의하기](#문의하기)

### 설명

/src 디렉토리에는 LibDriver MFRC522의 소스 파일이 포함되어 있습니다.

/interface 디렉토리에는 LibDriver MFRC522용 플랫폼 독립적인 IIC, SPI, UART버스 템플릿이 포함되어 있습니다.

/test 디렉토리에는 LibDriver MFRC522드라이버 테스트 프로그램이 포함되어 있어 칩의 필요한 기능을 간단히 테스트할 수 있습니다.

/example 디렉토리에는 LibDriver MFRC522프로그래밍 예제가 포함되어 있습니다.

/doc 디렉토리에는 LibDriver MFRC522오프라인 문서가 포함되어 있습니다.

/datasheet 디렉토리에는 MFRC522데이터시트가 있습니다.

/project 디렉토리에는 일반적으로 사용되는 Linux 및 마이크로컨트롤러 개발 보드의 프로젝트 샘플이 포함되어 있습니다. 모든 프로젝트는 디버깅 방법으로 셸 스크립트를 사용하며, 자세한 내용은 각 프로젝트의 README.md를 참조하십시오.

### 설치

/interface 디렉토리에서 플랫폼 독립적인 IIC, SPI, UART버스 템플릿을 참조하여 지정된 플랫폼에 대한 IIC, SPI, UART버스 드라이버를 완성하십시오.

/src 디렉토리, /interface 디렉토리 및 /example 디렉토리를 프로젝트에 추가하십시오.

### 사용

#### example basic

```C
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

### 문서

온라인 문서: https://www.libdriver.com/docs/mfrc522/index.html

오프라인 문서: /doc/html/index.html

### 기고

연락주세요lishifenging@outlook.com

### 저작권

저작권 (c) 2015 - 지금 LibDriver 판권 소유

MIT 라이선스(MIT)

이 소프트웨어 및 관련 문서 파일("소프트웨어")의 사본을 얻은 모든 사람은 이에 따라 무제한 사용, 복제, 수정, 통합, 출판, 배포, 2차 라이선스를 포함하여 소프트웨어를 처분할 수 있는 권리가 부여됩니다. 소프트웨어의 사본에 대한 라이선스 및/또는 판매, 그리고 소프트웨어가 위와 같이 배포된 사람의 권리에 대한 2차 라이선스는 다음 조건에 따릅니다.

위의 저작권 표시 및 이 허가 표시는 이 소프트웨어의 모든 사본 또는 내용에 포함됩니다.

이 소프트웨어는 상품성, 특정 목적에의 적합성 및 비침해에 대한 보증을 포함하되 이에 국한되지 않는 어떠한 종류의 명시적 또는 묵시적 보증 없이 "있는 그대로" 제공됩니다. 어떤 경우에도 저자 또는 저작권 소유자는 계약, 불법 행위 또는 기타 방식에 관계없이 소프트웨어 및 기타 소프트웨어 사용으로 인해 발생하거나 이와 관련하여 발생하는 청구, 손해 또는 기타 책임에 대해 책임을 지지 않습니다.

### 문의하기

연락주세요lishifenging@outlook.com