// #include "USB.h"
// #include "USBCDC.h"

// #include "pins.h"

// USBCDC USBSerial;
// #undef Serial
// #define Serial USBSerial

// void setup()
// {
//     USB.begin();
//     Serial.begin(115200);

//     pinMode(LED_RED, OUTPUT);
//     pinMode(LED_BLUE, OUTPUT);
//     pinMode(LED_GREEN, OUTPUT);
//     pinMode(LED_ORANGE, OUTPUT);
// }

// void loop()
// {
//     Serial.println("Hello!");
//     digitalWrite(LED_BLUE, HIGH);
//     delay(500);
//     digitalWrite(LED_BLUE, LOW);
//     delay(500);
//     Serial.println("Bye!");
//     delay(1000);
// }