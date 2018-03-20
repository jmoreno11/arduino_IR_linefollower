#include <QTRSensors.h>

#define Kp 0.08
#define Kd 0.1
#define Ki 0.003
#define MaxSpeed 70
#define BaseSpeed 70 
#define rightMotorPWM 11
#define leftMotorPWM 10
#define NUM_SENSORS  7   
#define buttonPin 12  
#define Led1 13

//Variables del loop
int lastError = 0;
int Ik = 0;
int Ika = 0;
int control = 0;
int flag;
int motorSpeed;
int buttonState;
int flag2;
int error;

//Funcion sensores

QTRSensorsRC qtrrc((unsigned char[]) {2,3,4,5,7,8,9} , NUM_SENSORS, 2500, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//Funcion para motores

void move (int, int);



void setup() //Inicializa
{
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  pinMode(Led1, OUTPUT); //Led1
  pinMode(buttonPin, INPUT); //Pulsador
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  delay(2000);


//////////////// C A L I B R A C I O N   D E L   S E N S O R //////////////// 

  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);


//////////////// F I N    D E    C A L I B R A C I O N //////////////// 
}



//////////////////////////

void loop() //Bucle principal
{
  unsigned int position = qtrrc.readLine(sensorValues);

// Hallamos el valor de posicion, que obtenemos tras leer los sensores
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]); 
    Serial.print('\t');
  }
  Serial.println(position); 
  
  delay(250);

  // Procedemos a calcular los parametros para el control

  error = position - 3000; /////////
//      Serial.print("error="); 
//      Serial.print(error); 
//      Serial.println();


//////////////// Control segun el boton //////////////// 
 
  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW)  
    flag2 = 1;

  if (buttonState == HIGH && flag2 == 1) {
    if (control < 6)
      control++;
    else
      control = 0;

    flag2 = 0;
    flag = 0;
  }
  
//////////////// Fin codigo boton //////////////// 



  switch (control) {

    case 1: //Control PID 
     //Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(1000);
      digitalWrite(Led1, LOW);
      delay(500);
      //

      if (flag = 0) {
        Ika = 0;
        Ik = 0;
        lastError = 0;
        flag = 1;
      }
      //////////////// Parametros //////////////// 
      Ik = Ika + error;
      motorSpeed = Kp * error + Kd * (error - lastError) + Ki * (Ik); // Salida controlada (sin BaseSpeed)
      Ika = Ik;
      lastError = error;
      break;
    case 2: // Control PD
    
     //Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(1000);
      
      //

      if (flag == 0) {
        lastError = 0;
        flag = 1;
      }
      
      //////////////// Parametros //////////////// 

      motorSpeed = Kp * error + Kd * (error - lastError); // Salida controlada (sin BaseSpeed)
      lastError = error;
      break;
    case 3: //Control PI
    
//Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      
      //
      if (flag == 0) {
        Ika = 0;
        Ik = 0;
        flag = 1;
      }
      
      //////////////// Parametros //////////////// 

      Ik = Ika + error;
      motorSpeed = Kp * error + Ki * (Ik); // Salida controlada (sin BaseSpeed)
      Ika = Ik;
      break;

    case 4: //Control P
    //Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      

      //////////////// Parametros //////////////// 
      
      motorSpeed = Kp * error; // Salida controlada (sin BaseSpeed)
      break;

    case 5: //Todo-Nada
  //Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      
      //////////////// Parametros //////////////// 
      
      if (error > 0) 
          motorSpeed = 120;

      else {
        if (error < 0)
          motorSpeed = -120;

      }

      break;

    case 6: //Histéresis
  //Indica caso
      delay(1000);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      digitalWrite(Led1, HIGH);
      delay(500);
      digitalWrite(Led1, LOW);
      delay(500);
      
//////////////// Parametros //////////////// 

      if ( error > 300)
        motorSpeed = 200;


      if (error < -300)
        motorSpeed = -200;
      break;

    case 0: //Paro
    //indica modo
      digitalWrite(Led1, HIGH);
      //
      analogWrite(leftMotorPWM, 0);
      analogWrite(rightMotorPWM, 0);
      break;
  }

 
//////////////// VELOCIDAD DE LOS MOTORES //////////////// 

//        Serial.print("motorSpeed="); 
//        Serial.print(motorSpeed); 
//        Serial.println();


  if (control != 0) {
    
    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;
    
    //////////////// Rango //////////////// 
    if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // Saturacion
    if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // Saturacion
    if (rightMotorSpeed < 0)rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)leftMotorSpeed = 0;
    //////////////// Fin Rango //////////////// 
    
    move(1, rightMotorSpeed);//
    move(0, leftMotorSpeed);//
  }
//////////////// FIN VELOCIDAD DE LOS MOTORES //////////////// 

}




////////////////////////////////////////////////////////////////






//////////////// FUNCION VELOCIDAD MOTOR //////////////// 

void move(int motor, int speed) { //, int direction

  if (motor == 0) {
    analogWrite(leftMotorPWM, speed);
  }
  if (motor == 1) {
    analogWrite(rightMotorPWM, speed);
  }
}

