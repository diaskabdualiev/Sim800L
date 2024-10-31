#include "HardwareSerial.h"

// Используем аппаратный UART1 для общения с SIM800L
HardwareSerial SIM800L(1);  // UART1
void signalIncomingCall();
// Определяем пины
#define RING_PIN 32    // Пин для RING
#define HOOK_PIN 33    // Пин для кнопки трубки (положил трубку)
#define DIAL_PIN 25    // Пин для дискового переключателя
#define MOTOR_PIN 27   // Пин для управления мотором через MOSFET
#define BUZZER_PIN 23  // Пин для управления зуммером

// Переменные для дискового переключателя
int needToPrint = 0;
int count = 0;
int in = DIAL_PIN;
int lastState = LOW;
int trueState = LOW;
long lastStateChangeTime = 0;
int cleared = 0;

// Константы для дискового переключателя
int dialHasFinishedRotatingAfterMs = 100; // Время после которого считаем, что цифра набрана
int debounceDelay = 10;                   // Антидребезг
long lastDigitTime = 0;                   // Время последней набранной цифры
long digitTimeout = 5000;                 // Таймаут для завершения набора номера (5 секунд)

// Переменные для хранения набранного номера
String dialedNumber = "";

// Переменные для отслеживания состояния трубки
bool callInProgress = false;
bool dialingStarted = false;  // Флаг начала набора номера

// Прототип функции
void readDial();

// Переменные для антидребезга HOOK_PIN
int hookState = HIGH; // Текущее устойчивое состояние пина HOOK_PIN
int lastHookReading = HIGH; // Последнее прочитанное значение с HOOK_PIN
unsigned long lastHookDebounceTime = 0; // Время последнего изменения состояния
unsigned long hookDebounceDelay = 50; // Задержка для антидребезга (мс)

// Переменные для управления мотором
bool incomingCall = false;
unsigned long ringStartTime = 0;
unsigned long motorSignalStartTime = 0;

// Переменные для обработки RING_PIN
int ringState = HIGH;
unsigned long ringLowStartTime = 0;

int buzzerChannel = 0; // Канал для PWM
int buzzerResolution = 8; // Разрешение PWM (8 бит)
int buzzerFrequency = 600; // Начальная частота зуммера (Гц)
void setup() {
  // Инициализация монитора порта
  Serial.begin(115200);
  while (!Serial) {
    ; // Ждем инициализацию Serial
  }

  Serial.println("Инициализация SIM800L...");

  // Инициализация UART1 для SIM800L
  SIM800L.begin(9600, SERIAL_8N1, 16, 17);

  // Пауза для инициализации модуля
  delay(5000);
  Serial.println("SIM800L готов к приему AT-команд.");

  // Настройка пинов
  pinMode(RING_PIN, INPUT_PULLUP);
  pinMode(HOOK_PIN, INPUT_PULLUP); // Предполагаем, что при поднятой трубке пин в состоянии LOW
  pinMode(DIAL_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT); // Настраиваем MOTOR_PIN как выход
  digitalWrite(MOTOR_PIN, LOW); // Выключаем мотор по умолчанию

  ledcSetup(buzzerChannel, buzzerFrequency, buzzerResolution);
  ledcAttachPin(BUZZER_PIN, buzzerChannel);
  ledcWrite(buzzerChannel, 0); // Выключаем зуммер по умолчанию

  // Инициализация переменных дискового переключателя
  lastState = digitalRead(DIAL_PIN);
  trueState = lastState;

  // Инициализация HOOK_PIN
  hookState = digitalRead(HOOK_PIN);
  lastHookReading = hookState;

  // Инициализация RING_PIN
  ringState = digitalRead(RING_PIN);

  // Дополнительный вывод
  Serial.println("Инициализация завершена.");
}

void loop() {
  // Обработка RING_PIN для обнаружения входящего вызова
  int currentRingState = digitalRead(RING_PIN);

  if (currentRingState != ringState) {
    ringState = currentRingState;

    if (ringState == LOW) {
      ringLowStartTime = millis();
    } else if (ringState == HIGH) {
      // RING_PIN перешел из LOW в HIGH
      if (incomingCall && !callInProgress) {
        // Вызывающий абонент сбросил вызов до ответа
        incomingCall = false;
        digitalWrite(MOTOR_PIN, LOW);
        Serial.println("Вызывающий абонент сбросил вызов до ответа.");
      }
      ringLowStartTime = 0;
    }
  } else if (ringState == LOW && ringLowStartTime > 0) {
    unsigned long ringDuration = millis() - ringLowStartTime;
    if (ringDuration >= 2000 && !incomingCall) { // Если LOW более 2 секунд
      incomingCall = true;
      motorSignalStartTime = millis();
      Serial.println("Обнаружен реальный входящий вызов!");
    }
  }

  // Управление мотором для сигнализации входящего вызова
  if (incomingCall) {
    // Проверяем, не поднята ли трубка
    if (hookState == LOW) {
      // Трубка поднята, прекращаем сигнализацию
      incomingCall = false;
      digitalWrite(MOTOR_PIN, LOW);
      // Ответ на вызов
      SIM800L.println("ATA"); // Ответ на вызов
      Serial.println("Ответ на входящий вызов...");
      callInProgress = true;
    } else {
      // Управляем мотором
      signalIncomingCall();
    }
  } else {
    // Выключаем мотор, если нет входящего вызова
    digitalWrite(MOTOR_PIN, LOW);
  }

  // Чтение данных от SIM800L и вывод в монитор порта
  if (SIM800L.available()) {
    while (SIM800L.available()) {
      char c = SIM800L.read();
      Serial.print(c);
    }
  }

  // Чтение команд из монитора порта
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();

    if (command.equalsIgnoreCase("REST")) {
      // Функция перезагрузки модуля (если необходимо)
      Serial.println("Перезагрузка модуля SIM800L...");
      // Добавьте здесь логику перезагрузки, если нужно
    } else {
      SIM800L.println(command);
    }
  }

  // Антидребезг для HOOK_PIN
  int reading = digitalRead(HOOK_PIN);

  if (reading != lastHookReading) {
    lastHookDebounceTime = millis();
  }

  if ((millis() - lastHookDebounceTime) > hookDebounceDelay) {
    if (reading != hookState) {
      hookState = reading;

      if (hookState == LOW) {
        // Трубка поднята.
        Serial.println("Трубка поднята.");

        if (incomingCall) {
          // Ответ на входящий вызов
          incomingCall = false;
          digitalWrite(MOTOR_PIN, LOW);
          SIM800L.println("ATA"); // Ответ на вызов
          Serial.println("Ответ на входящий вызов...");
          callInProgress = true;
        } else {
          Serial.println("Ожидание набора номера...");
          dialingStarted = false;  // Сбрасываем флаг начала набора
          ledcWriteTone(buzzerChannel, buzzerFrequency);  // Включаем зуммер с заданной частотой
          ledcWrite(buzzerChannel, 128); // Устанавливаем громкость (скважность)
        }
      } else {
        // Трубка положена.
        Serial.println("Трубка положена.");
        ledcWriteTone(buzzerChannel, 0); // Выключаем зуммер
        if (callInProgress) {
          SIM800L.println("ATH"); // Завершаем вызов
          Serial.println("Завершаем вызов...");
          callInProgress = false;
        }
        // Сброс набранного номера
        dialedNumber = "";
      }
    }
  }

  lastHookReading = reading;

  // Проверяем, поднята ли трубка после антидребезга
  if (hookState == LOW && !incomingCall) {
    // Трубка поднята, и нет входящего вызова, принимаем ввод
    readDial();

    // Проверка на завершение набора номера
    if (dialedNumber.length() > 0 && (millis() - lastDigitTime > digitTimeout)) {
      Serial.print("Набран номер: ");
      Serial.println(dialedNumber);

      // Инициируем вызов
      String atCommand = "ATD" + dialedNumber + ";";
      SIM800L.println(atCommand);
      Serial.println("Инициируем вызов...");
      callInProgress = true;

      // Сброс набранного номера
      dialedNumber = "";
    }
  } else {
    // Если трубка не поднята или есть входящий вызов, зуммер выключен
    ledcWriteTone(buzzerChannel, 0); // Выключаем зуммер
  }
}

// Функция для чтения дискового переключателя
void readDial() {
  int reading = digitalRead(DIAL_PIN);

  if ((millis() - lastStateChangeTime) > dialHasFinishedRotatingAfterMs) {
    if (needToPrint) {
      int digit = count % 10;
      if (digit == 0) {
        digit = 0; // При 10 импульсах цифра '0'
      }
      dialedNumber += String(digit);
      Serial.print("Набрана цифра: ");
      Serial.println(digit);
      needToPrint = 0;
      count = 0;
      cleared = 0;
      lastDigitTime = millis();
    }
  }

  if (reading != lastState) {
    lastStateChangeTime = millis();
  }
  if ((millis() - lastStateChangeTime) > debounceDelay) {
    if (reading != trueState) {
      trueState = reading;
      if (trueState == HIGH) {
        count++;
        needToPrint = 1;
        Serial.print("Обнаружен импульс. Текущее количество: ");
        Serial.println(count);

        if (!dialingStarted) {
          dialingStarted = true;  // Устанавливаем флаг начала набора
          ledcWriteTone(buzzerChannel, 0);  // Выключаем зуммер
        }
      }
    }
  }
  lastState = reading;
}

// Функция для сигнализации входящего вызова
void signalIncomingCall() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - motorSignalStartTime;

  if (elapsedTime % 6000 < 3000) {
    // В течение первых 3 секунд сигнализируем
    if ((elapsedTime / 30) % 2 == 0) {
      digitalWrite(MOTOR_PIN, HIGH); // Включаем мотор
    } else {
      digitalWrite(MOTOR_PIN, LOW); // Выключаем мотор
    }
  } else {
    // Следующие 3 секунды ничего не делаем (мотор выключен)
    digitalWrite(MOTOR_PIN, LOW);
  }
}
