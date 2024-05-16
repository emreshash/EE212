Giriş
Bu proje, su seviyesini izleyen ve gerektiğinde servo motorlar yardımıyla su akışını kontrol eden bir akıllı baraj sistemi tasarlamak için gerçekleştirilmiştir. Proje, bir Freescale/NXP KL25Z mikrodenetleyici kartı kullanılarak C dili ile yazılmıştır.

Kullanılan Malzemeler
Freescale/NXP KL25Z Mikrodenetleyici Kartı: Projenin ana kontrol birimidir. ARM Cortex-M0+ çekirdeğine sahip olan bu mikrodenetleyici, düşük güç tüketimi ve yüksek performans sunar.
Servo Motorlar: Baraj kapaklarını açmak ve kapatmak için kullanılır. Projede iki adet servo motor kullanılmıştır.
Su Seviye Sensörleri (ADC Kanalları): Su seviyesini ölçmek için kullanılır. Analog sinyaller dijital değerlere dönüştürülerek mikrodenetleyici tarafından işlenir.
Butonlar (Anahtarlar): Kullanıcı tarafından manuel kontrol sağlamak için kullanılır. İki buton servo motorların manuel kontrolünü, bir buton ise güvenli su seviyesini belirler.
Güç Kaynağı: Tüm bileşenlerin çalışması için gerekli olan gücü sağlar.
Sistem Tasarımı ve Çalışma Prensibi
Mikrodenetleyici: KL25Z
KL25Z mikrodenetleyici kartı, ARM Cortex-M0+ çekirdeği üzerine inşa edilmiştir ve aşağıdaki özelliklere sahiptir:

48 MHz saat hızı
128 KB flash bellek
16 KB RAM
Çok sayıda GPIO pinleri
PWM (Pulse Width Modulation) desteği
ADC (Analog to Digital Converter) desteği
C Dili
Proje, düşük seviye donanım erişimine olanak tanıyan C dili ile yazılmıştır. C dili, bellek yönetimi ve donanım erişimi konusunda sağladığı esneklik ile gömülü sistem projeleri için ideal bir programlama dilidir.

Akıllı Barajın Tanıtımı
Akıllı baraj sistemi, su seviyesini sürekli olarak izleyen ve bu seviyeye göre baraj kapaklarını açıp kapatan bir kontrol sistemidir. Sistem, su seviyesi belirli bir güvenli seviyeyi aştığında kapakları açarak suyun boşaltılmasını sağlar ve su seviyesi düştüğünde kapakları kapatarak suyu tutar. Ayrıca, manuel kontrol butonları ile kullanıcı, servo motorları manuel olarak da kontrol edebilir.

Sistem Bileşenleri ve Fonksiyonları
PWM (Pulse Width Modulation) Yapılandırması
PWM, servo motorların konumlarını kontrol etmek için kullanılır. 50 Hz frekansta çalışan PWM sinyalleri, servo motorların açısını belirlemek için kullanılır. PWM sinyalleri, %5 ila %10 arasında değişen görev döngüsü ile kontrol edilir.

%5 Duty Cycle: Servo motor kapalı konumda.
%10 Duty Cycle: Servo motor açık konumda.
ADC (Analog to Digital Converter) Yapılandırması
ADC modülü, su seviye sensörlerinden gelen analog sinyalleri dijital değerlere dönüştürmek için kullanılır. Bu değerler, su seviyesinin belirli bir güvenli seviyeyi aşıp aşmadığını belirlemek için işlenir.

GPIO (General-Purpose Input/Output) Yapılandırması
GPIO pinleri, butonların durumunu okumak ve bu durumlara göre servo motorları kontrol etmek için kullanılır. Butonlara bağlı olan pinler giriş olarak yapılandırılmıştır ve bu pinler üzerinden gelen sinyallerle kesme (interrupt) işlemleri gerçekleştirilir.

Kesme (Interrupt) İşlemleri
Kesme işlemleri, butonların durumunu izlemek ve butona basıldığında gerekli işlemleri gerçekleştirmek için kullanılır. Her buton, belirli bir servo motorun manuel kontrolünü sağlar. Ayrıca, güvenli su seviyesi butonu, su seviyesinin hangi yüzdeye ulaşması gerektiğini belirler.
Kod Açıklaması




Projenin kodu, yukarıda açıklanan bileşenleri ve fonksiyonları içeren aşağıdaki ana bölümlerden oluşmaktadır:

PWM Yapılandırması:
c
Copy code
void InitPWM(void) {
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; // TPM0 saatini etkinleştir
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // TPM saat kaynağını PLLFLLCLK olarak ayarla

    TPM0->SC = TPM_SC_PS(7); // Prescaler 128
    TPM0->MOD = 37500; // 50Hz için mod değeri

    TPM0->CONTROLS[BOARD_TPM_CHANNEL_SERVO1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
    TPM0->CONTROLS[BOARD_TPM_CHANNEL_SERVO2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

    TPM0->CONTROLS[BOARD_TPM_CHANNEL_SERVO1].CnV = (TPM0->MOD * SERVO_CLOSE_DUTY_CYCLE) / 100;
    TPM0->CONTROLS[BOARD_TPM_CHANNEL_SERVO2].CnV = (TPM0->MOD * SERVO_CLOSE_DUTY_CYCLE) / 100;

    TPM0->SC |= TPM_SC_CMOD(1); // TPM sayacını başlat
}
ADC Yapılandırması:
c
Copy code
void InitADC(void) {
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; // ADC0 saatini etkinleştir

    ADC0->CFG1 = ADC_CFG1_ADIV(3) | ADC_CFG1_MODE(1); // ADIV: 8, MODE: 12-bit
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK; // Yazılım tetikleme kullan

    // ADC kalibrasyonu
    ADC0->SC3 = ADC_SC3_CAL_MASK;
    while (ADC0->SC3 & ADC_SC3_CAL_MASK) { } // Kalibrasyon tamamlanana kadar bekle
}
GPIO Yapılandırması:
c
Copy code
void InitGPIO(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK; // PORTA saatini etkinleştir
    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; // PORTD saatini etkinleştir

    PORTD->PCR[SW1_PIN_INT_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTA->PCR[SW2_PIN_INT_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PORTA->PCR[SW3_PIN_INT_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);

    PTD->PDDR &= ~(1 << SW1_PIN_INT_PIN); // SW1 pini giriş
    PTA->PDDR &= ~(1 << SW2_PIN_INT_PIN); // SW2 pini giriş
    PTA->PDDR &= ~(1 << SW3_PIN_INT_PIN); // SW3 pini giriş

    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTD_IRQn);
}
Servo Kontrol Fonksiyonu:
c
Copy code
void vServo_Control(uint8_t chnlNumber, uint8_t chnl, uint8_t dutyCyclePercent) {
    TPM0->CONTROLS[chnl].CnV = (TPM0->MOD * dutyCyclePercent) / 100;
}
ADC Kesme Fonksiyonu:
c
Copy code
void ADC0_IRQHandler(void) {
    if (ADC0->SC1[0] & ADC_SC1_COCO_MASK) {
        if (Level_Channel_No == ADC16_LEVEL_SENSE_CH1) {
            g_Adc16ConversionValue[0] = ADC0->R[0];
            if (!Servo1_State) {
                if (g_Adc16ConversionValue[0] > ((u8SafeLevelPercentage * 4096) / 100)) {
                    if (!Servo1_Open) {
                        vServo_Control(BOARD_TPM_CHANNEL_SERVO1, 0, SERVO_OPEN_DUTY_CYCLE);
                        Servo1_Open = true;
                    }
                } else if (g_Adc16ConversionValue[0] < (((u8SafeLevelPercentage - 10) * 4096) / 100)) {
                    if (Servo1_Open) {
                        vServo_Control(BOARD_TPM_CHANNEL_SERVO1, 0, SERVO_CLOSE_DUTY_CYCLE);
                        Servo1_Open = false;
                    }
                }
            }
            Level_Channel_No = ADC16_LEVEL_SENSE_CH2;
        } else if (Level_Channel_No == ADC16_LEVEL_SENSE_CH2) {
            g_Adc16ConversionValue[1] = ADC0->R[0];
            if (!Servo2_State) {
                if (g_Adc16ConversionValue[1] > ((u8SafeLevelPercentage * 4096) / 100)) {
                    if (!Servo2_Open) {
                        vServo_Control(BOARD_TPM_CHANNEL_SERVO2, 1, SERVO_OPEN_DUTY_CYCLE);
                        Servo2_Open = true;
                    }
                } else if (g_Adc16ConversionValue[1] < (((u8SafeLevelPercentage - 10) * 4096) / 100)) {
                    if (Servo2_Open) {
                        vServo_Control(BOARD_TPM_CHANNEL_SERVO2, 1, SERVO_CLOSE_DUTY_CYCLE);
                        Servo2_Open = false;
                    }
                }
            }
            Level_Channel_No = ADC16_LEVEL_SENSE_CH1;
        }
        ADC0->SC1[0] = ADC_SC1_ADCH(Level_Channel_No);
    }
}
GPIO Kesme Fonksiyonları:
c
Copy code
void PORTA_IRQHandler(void) {
    if (PORTA->ISFR & (1 << SW2_PIN_INT_PIN)) {
        if (PTA->PDIR & (1 << SW2_PIN_INT_PIN)) {
            vServo_Control(BOARD_TPM_CHANNEL_SERVO1, 0, SERVO_OPEN_DUTY_CYCLE);
            Servo1_State = true;
        } else {
            vServo_Control(BOARD_TPM_CHANNEL_SERVO1, 0, SERVO_CLOSE_DUTY_CYCLE);
            Servo1_State = false;
        }
        PORTA->ISFR |= (1 << SW2_PIN_INT_PIN); // Bayrakları temizle
    }
    if (PORTA->ISFR & (1 << SW3_PIN_INT_PIN)) {
        if (PTA->PDIR & (1 << SW3_PIN_INT_PIN)) {
            vServo_Control(BOARD_TPM_CHANNEL_SERVO2, 1, SERVO_OPEN_DUTY_CYCLE);
            Servo2_State = true;
        } else {
            vServo_Control(BOARD_TPM_CHANNEL_SERVO2, 1, SERVO_CLOSE_DUTY_CYCLE);
            Servo2_State = false;
        }
        PORTA->ISFR |= (1 << SW3_PIN_INT_PIN); // Bayrakları temizle
    }
}

void PORTD_IRQHandler(void) {
    if (PORTD->ISFR & (1 << SW1_PIN_INT_PIN)) {
        u8SafeLevelPercentage = (PTD->PDIR & (1 << SW1_PIN_INT_PIN)) ? 50 : 75;
        PORTD->ISFR |= (1 << SW1_PIN_INT_PIN); // Bayrakları temizle
    }
}
Ana Fonksiyon (main):
c
Copy code
int main(void) {
    InitPWM();
    InitADC();
    InitGPIO();

    ADC0->SC1[0] = ADC_SC1_ADCH(Level_Channel_No); // ADC dönüşümünü başlat

    while (1) {
        // Ana döngü
    }
}
Sonuç
Bu proje, su seviyesini izleyerek ve kontrol ederek bir barajın güvenli bir şekilde çalışmasını sağlamak amacıyla tasarlanmıştır. Freescale/NXP KL25Z mikrodenetleyici ve C dili kullanılarak gerçekleştirilen bu sistem, barajın otomatik ve manuel kontrolünü mümkün kılmaktadır. Bu sayede su seviyesi güvenli sınırlar içinde tutulmakta ve taşkın riskleri minimize edilmektedir.






