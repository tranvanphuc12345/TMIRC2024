// setup thông số đk DC
const int offsetA = 1;
const int offsetB = 1;
// Khai báo địa chỉ chân nano with Tb6612
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9
//
void setup() {
  //set pin 
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
}
void loop() {
  digitalWrite(STBY, HIGH);
  // Chạy thằng 
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 255);
  // chạy lùi
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255);
    // rẻ phải
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, 255);
  // rẻ trái
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 255);
}
///////////////////////////////////////////////////////////////////////////////

const int pwmA = 3;   // PWM điều khiển động cơ A
const int dirA = 4;   // Điều khiển hướng quay của động cơ A

const int pwmB = 5;   // PWM điều khiển động cơ B
const int dirB = 6;   // Điều khiển hướng quay của động cơ B

const int standby = 7; // Chân Standby để bật/tắt modul

void setup() {
  // Cấu hình các chân
  pinMode(pwmA, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(standby, OUTPUT);
  // Bật modul TB6612
  digitalWrite(standby, HIGH); // bật lên mức cao ms chạy 
}
void Dithang() {
  digitalWrite(dirA, HIGH); // Động cơ A quay về phía trước
  digitalWrite(dirB, HIGH); // Động cơ B quay về phía trước
  analogWrite(pwmA, 128);   // Tốc độ 50% cho động cơ A
  analogWrite(pwmB, 128);   // Tốc độ 50% cho động cơ B
}

void Dilui() {
  digitalWrite(dirA, LOW);  // Động cơ A quay ngược lại
  digitalWrite(dirB, LOW);  // Động cơ B quay ngược lại
  analogWrite(pwmA, 128);   // Tốc độ 50% cho động cơ A
  analogWrite(pwmB, 128);   // Tốc độ 50% cho động cơ B
}

void qoetrai() {
  digitalWrite(dirA, LOW);  // Động cơ A quay ngược lại
  digitalWrite(dirB, HIGH); // Động cơ B quay về phía trước
  analogWrite(pwmA, 128);   // Tốc độ 50% cho động cơ A
  analogWrite(pwmB, 128);   // Tốc độ 50% cho động cơ B
}

void qoephai() {
  digitalWrite(dirA, HIGH); // Động cơ A quay về phía trước
  digitalWrite(dirB, LOW);  // Động cơ B quay ngược lại
  analogWrite(pwmA, 128);   // Tốc độ 50% cho động cơ A
  analogWrite(pwmB, 128);   // Tốc độ 50% cho động cơ B
}

void dung() {
  analogWrite(pwmA, 0);     // Tốc độ 0% cho động cơ A
  analogWrite(pwmB, 0);     // Tốc độ 0% cho động cơ B
}

void loop() {
  Dithang();
  Dilui();
  qoephai();
  qoetrai();
  dung();
}
///////////////////////////////////////////////////////////////////////////

int standBy = 10;

// Motor A
int PWMA = 3;  
int AIN1 = 9;  
int AIN2 = 8;  

// Motor B
int PWMB = 5;  /
int BIN1 = 11; 
int BIN2 = 12; 
void setup() {
    
    pinMode(standBy, OUTPUT);
    
    // Motor A
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    
    // Motor B
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    
    Serial.begin(19200);
    Serial.println("................");
}

void loop() {
    Tien(128);        
    delay(2000);        
    stop();            
    delay(250);         
    
    Lui(255);        
    delay(2000);         
    stop();             
    delay(250);         
    
    Trai(64);        
    delay(2000);         
    stop();              
    delay(250);          
    
    Phai(255);     
    delay(2000);         
    stop();              
    delay(2000);         
}
/////////////////////////////////
void runMotor(int motor, int spd, int dir) {
    digitalWrite(standBy, HIGH); 
    
    boolean dirPin1 = LOW;   // Biến boolean để thiết lập hướng của động cơ
    boolean dirPin2 = HIGH;
  // Nếu hướng là 0 (lùi), đổi giá trị của dirPin1 và dirPin2
    if(dir == 0) {     
        dirPin1 = HIGH;  
        dirPin2 = LOW;
    }
  // Kiểm tra động cơ nào đang được điều khiển
    if(motor == 0) {   //A
        digitalWrite(AIN1, dirPin1);
        digitalWrite(AIN2, dirPin2);
        analogWrite(PWMA, spd);
    } else if(motor == 1) {   //B
        digitalWrite(BIN1, dirPin1);
        digitalWrite(BIN2, dirPin2);
        analogWrite(PWMB, spd);
    }
}
//
void Tien(int spd) {
    runMotor(0, spd, 1);
    runMotor(1, spd, 1);
}

void Lui(int spd) {
    runMotor(0, spd, 0);
    runMotor(1, spd, 0);
}

void Trai(int spd) {
    runMotor(0, spd, 0);
    runMotor(1, spd, 1);
}

void Phai(int spd) {
    runMotor(0, spd, 1);
    runMotor(1, spd, 0);
}

void Dung() {
    runMotor(0, 0, 0);
    runMotor(1, 0, 0);
}



