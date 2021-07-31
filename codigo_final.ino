/**
 *  Authors:
 *      Arias Gómez-Calcerrada, José Joaquín
 *      Corbacho Sánchez, Manuel Jesús
 *  Arduino model: Leonardo
*/

/**
* Wheels on the back of the robot, ball in the front
* Control of the floor using 3 CNY70 sensors
* 1 in the front.
* 2 in the back
*/
int start;

/*Constantes y variables*/
const float ResolutionADC=0.00488; //4.88mV
float distance;
unsigned long time_bounce;
const int tiempo_giro=700;
int Value_SharpR=0;
int Value_SharpL=0;
float VoltageR, VoltageL;
float Voltage_CNY_Back_L=0.0f;
float Voltage_CNY_Back_R=0.0f;
float Voltage_CNY_Front=0.0f;
int tiempo=0;
int casillas=0;
//
int antPin2=1;
int antPin3=1;
int transicionPin2=0;
int transicionPin3=0;
//
int contadorCeldas=0;
bool espera=false;
bool derecha=false;
bool senal_derecha=false;
bool senal_delante=true;
int ultimo_movimiento=0; //-1->No asignado 0->Arriba 1->Abajo 2->Izquierda 3->Derecha
int ultimo=0;            //-1->No asignado 0->Arriba 1->Abajo 2->Izquierda 3->Derecha
int indice=0;
int* orientacion;
int* movimientos;
bool terminar=false;
int diferencia;
int estado=1; /* 1->Dentro de una casilla; 2->Tocando con sensor del frente; 3->Tocando con sensor de atrás */
//Prueba
bool parar=false;
//
int Value_CNY_Back_L=0;
int Value_CNY_Back_R=0;
int Value_CNY_Front=0;

/* CNY pins */
const int CNY_Back_L=A0;
const int CNY_Back_R=A1;
const int CNY_Front=A5;
/* Motor pins */
const int pin1Left_Motor= 9;
const int pin2Left_Motor=10;
const int pin1RightMotor= 5;
const int pin2RightMotor= 6;
/* Sharp pins */
const int SharpR=A3;
const int SharpL=A8;
/* Ultrasonic pins */
const int Trigger=11;
const int Echo=7;

void stop() {
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
}

void stopRight() {
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin2RightMotor, 0);
}

void stopLeft() {
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1Left_Motor, 0);
}

void forward(int AnalogValue) {
    stop();
    //Suponiendo una diferencia rotacional entre ruedas del 30%
    diferencia=0.2*AnalogValue;
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, AnalogValue-diferencia);
    analogWrite(pin2RightMotor, AnalogValue);
}

void tackRight() {
    /*diferencia=0.2*AnalogValue;*/
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 128);
    analogWrite(pin2RightMotor, 255);
}

void tackLeft() {
    /*diferencia=0.2*AnalogValue;*/
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 255);
    analogWrite(pin2RightMotor, 128);
}

void back(int AnalogValue) {
    stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, AnalogValue);
    analogWrite(pin1RightMotor, AnalogValue);
}

void backLeft() {
    stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, 255);
    analogWrite(pin1RightMotor, 0);
}

void backRight() {
    stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 255);
}

bool lineFoundFront() {
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation
    Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;

    bool found=false;
    if(Voltage_CNY_Front>3.4 /*&& (Voltage_CNY_Front-min(Voltage_CNY_Back_L, Voltage_CNY_Back_R))>1*/) {  //Valor estaba a 3.4
      found=true;
    }
    return found;
}

bool lineFoundBack() {
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation
    Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;

    bool found=false;
    if((Voltage_CNY_Back_L>3.7 || Voltage_CNY_Back_R>3.7) /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/) { //Voltage_CNY_Back_L y R antes estaban a 3.4
      found=true;
    }
    return found;
}

bool lineFoundBack_L() {
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation
    Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;

    bool found=false;
    if(Voltage_CNY_Back_L>3.7 /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/) {   //Voltage_CNY_Back_L estaba a 3.4
      found=true;
    }
    return found;
}

bool lineFoundBack_R() {
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation
    Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;

    bool found=false;
    if(Voltage_CNY_Back_R>3.7 /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/) {   //Voltage_CNY_Back_R estaba a 3.4
      found=true;
    }
    return found;
}

bool todoNegro() {
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation
    Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;

    bool found=false;
    if(Voltage_CNY_Front>3.4 && Voltage_CNY_Back_L>3.7 && Voltage_CNY_Back_R>3.7) {   //Los voltajes estaban a 3.4
      found=true;
    }
    return found;
}

bool noLineFound() {
  return !lineFoundFront() && !lineFoundBack();
}

double getDistance(double valor) {
  double centimetros=0;
  if(valor<0.5 || valor>2.7) {
    return centimetros;
  }
  else {
    centimetros=valor/13.05;
    centimetros=(1-centimetros*0.42)/centimetros;
    return centimetros;
  }
}

void showSharpR() {
  Value_SharpR=analogRead(SharpR);
  VoltageR=Value_SharpR*ResolutionADC;
  /*Serial.println(Value_SharpR);
  Serial.print(" Voltage: ");
  Serial.print(Voltage);
  Serial.println(" V");*/
  Serial.print("Sharp R: ");
  Serial.print(getDistance(VoltageR));
  Serial.println(" cm");
}

void showSharpL() {
  Value_SharpL=analogRead(SharpL);
  VoltageL=Value_SharpL*ResolutionADC;
  /*Serial.println(Value_SharpR);
  Serial.print(" Voltage: ");
  Serial.print(VoltageL);
  Serial.println(" V");*/
  Serial.print("Sharp L: ");
  Serial.print(getDistance(VoltageL));
  Serial.println(" cm");
}

double getSharpR() {
  Value_SharpR=analogRead(SharpR);
  VoltageR=Value_SharpR*ResolutionADC;
  return getDistance(VoltageR);
}

double getSharpL() {
  Value_SharpL=analogRead(SharpL);
  VoltageL=Value_SharpL*ResolutionADC;
  return getDistance(VoltageL);
}

double getUltrasonic() {
  digitalWrite(Trigger, LOW);
  delayMicroseconds(5);
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trigger, LOW);
  time_bounce=pulseIn(Echo, HIGH);
  distance = 0.017 * time_bounce; //Fórmula para calcular la distancia
  return distance;
}

void setup(){
    pinMode(pin1Left_Motor, OUTPUT);
    pinMode(pin2Left_Motor, OUTPUT);
    pinMode(pin1RightMotor, OUTPUT);
    pinMode(pin2RightMotor, OUTPUT);
    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(2,INPUT);
    pinMode(3,INPUT);
    orientacion=new int[25]; //25 casillas como máximo de recorrido.
    for(int i=0; i<50; i++) {
      orientacion[i]=-1;  //Empiezan sin asignar
    }
    movimientos=new int[25]; //25 casillas como máximo de recorrido.
    for(int i=0; i<50; i++) {
      movimientos[i]=-1;  //Empiezan sin asignar
    }
    Serial1.begin(9600);
    start=0;
}


bool hasPassedBox() {
  int estado_anterior=estado;
  //Actualizar estado
  switch(estado) {
    case 1: if(lineFoundFront()) {
              estado=2;
            } break;
    case 2: if(lineFoundBack() && !lineFoundFront()) {
              estado=3;
            } break;
    case 3: if(!lineFoundBack()) {
              estado=1;
            } break;
  }
  //Fin actualizar estado
  if(estado_anterior==3 && estado==1) { Serial1.println('m'); return true;}
  else {return false;}
}

void balanceLeft() {  /* Mantener el robot con la referencia de la pared de la izquierda */
  double distL=getSharpL();
  if(distL<10) {   //Valía 6
    //Serial1.println("VIRANDO A LA DERECHA");
    tackRight();
    delay(50);
    stop();
  }
  else {
    //Serial1.println("VIRANDO A LA IZQUIERDA");
    tackLeft();
    delay(50);
    stop();
  }
}

void balanceRight() {  /* Mantener el robot con la referencia de la pared de la derecha */
  double distR=getSharpR();
  if(distR<10) {   //Valía 6
    tackLeft();
    delay(50);
    stop();
  }
  else {
    tackRight();
    delay(50);
    stop();
  }
}

void intelligentForward(int potencia=150) {
  double distR=getSharpR();
  double distL=getSharpL();
  double diferencia=distR-distL;
  double umbralp=2;
  double umbraln=-umbralp;
  double topeR=12;
  double topeL=12;
  
  if(distR>topeR || distR==0 || distL>topeL || distL==0) {
    if((distR>topeR || distR==0) && (distL>topeL || distL==0)) {
      forward(potencia);
    }
    if((distR>topeR || distR==0) && !(distL>topeL || distL==0)) {
      //Serial1.println("BALANCEANDO A LA IZQUIERDA");
      balanceLeft();
      forward(potencia);
    }
    if(!(distR>topeR || distR==0) && (distL>topeL || distL==0)) {
      //Serial1.println("BALANCEANDO A LA DERECHA");
      balanceRight();
      forward(potencia);
    }
  }
  else {
    if(diferencia>umbralp) {
      tackRight();
      delay(50);
      stop();
    }
    else {
      if(diferencia<umbraln) {
        tackLeft();
        delay(50);
        stop();
      }
      else {
        forward(potencia);
      }
    }
  }
}

void alinear() {
  stop();
  bool alineado=false;
  while(!alineado) {
    if(!lineFoundBack()) {
      //Ir atrás
      back(255);
    }
    else {
      if(lineFoundBack_R() && lineFoundBack_L()) {
        //Ya está alineado
        alineado=true;
        stop();
      }
      else {
        if(lineFoundBack_L()) {
          //Mover culo atrás a la izquierda
          Serial1.println("YENDO ATRAS POR LA IZQUIERDA");
          //Read CNY values
          Value_CNY_Back_L = analogRead(CNY_Back_L);
      
          //Voltage calculation1
          Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
          Serial1.println("Valor ATRAS_L: "); Serial1.println(Voltage_CNY_Back_L);
          backLeft();
        }
        if(lineFoundBack_R()) {
          //Mover culo atrás a la derecha
          backRight();
        }
      }
    }
  }
}

bool revolucionesPin2(int transiciones) {
  transicionPin2=0;
  while(true) {
    //Serial1.println("VALOR DEL PIN 2: "); Serial1.println(digitalRead(2));
    int Pin2=digitalRead(2);
    if((antPin2==1 && Pin2==0) || (antPin2==0 && Pin2==1)) {
      if(antPin2==1 && Pin2==0) {
        antPin2=0; 
      }
      else {
        antPin2=1;
      }
      //Actualizar transicionPin2
      ++transicionPin2;
      if(transicionPin2==transiciones) {
        return true;
      }
    }
  }
}

bool revolucionesPin3(int transiciones) {
  transicionPin3=0;
  while(true) {
    //Serial1.println("VALOR DEL PIN 3: "); Serial1.println(digitalRead(3));
    int Pin3=digitalRead(3);
    if((antPin3==1 && Pin3==0) || (antPin3==0 && Pin3==1)) {
      if(antPin3==1 && Pin3==0) {
        antPin3=0; 
      }
      else {
        antPin3=1;
      }
      //Actualizar transicionPin2
      ++transicionPin3;
      if(transicionPin3==transiciones) {
        return true;
      }
    }
  }
}

void rotarDerecha(int iteraciones=10) {
  Serial1.println('r');
  for(int a=0; a<iteraciones; a++) {
    analogWrite(pin2Left_Motor, 150);
    while(!revolucionesPin3(1)) {}
    analogWrite(pin2RightMotor, 150);
    while(!revolucionesPin2(1)) {}
  }
  stop();
}

void rotarIzquierda(int iteraciones=10) {
  Serial1.println('l');
  for(int a=0; a<iteraciones; a++) {
    analogWrite(pin1Left_Motor, 150);
    while(!revolucionesPin3(1)) {}
    analogWrite(pin1RightMotor, 150);
    while(!revolucionesPin2(1)) {}
  }
  stop();
}

void atras(int iteraciones=7) {
  for(int a=0; a<iteraciones; a++) {
    bool pararIzquierda=false;
    bool pararDerecha=false;
    if(pararIzquierda) {
      stopLeft();
    }
    else {
      //Mover rueda izquierda durante x revoluciones
      analogWrite(pin2Left_Motor, 255);
      if(revolucionesPin3(1)) {
        pararIzquierda=true;
      }
    }
    if(pararDerecha) {
      stopRight();
    }
    else {
      //Mover rueda derecha durante x revoluciones
      analogWrite(pin1RightMotor, 255);
      if(revolucionesPin2(1)) {
        pararDerecha=true;
      }
    }
  }
  stop();
}

void rotar180() {
  //Serial1.println('b');
  rotarIzquierda(6);
  atras();
  rotarIzquierda(17);
  estado=2;
}

int actualizar_movimiento(int ultimo_movimiento, int movimiento) {
    // 0 up, 1 down, 2 left, 3 right
    if(ultimo_movimiento < 0 || ultimo_movimiento > 3) 
        return -1;
    if( ultimo_movimiento == 0) 
        return movimiento; 
    int nuevo_movimiento[3][4] = {  
        { 1, 0, 3, 2},
        { 2, 3, 1, 0},
        { 3, 2, 0, 1} 
    };
    return nuevo_movimiento[ultimo_movimiento-1][movimiento];
}

bool opuestos(int movimiento1, int movimiento2) {
    switch(movimiento1) {
        case 0: return (movimiento2==1); break;
        case 1: return (movimiento2==0); break;
        case 2: return (movimiento2==3); break;
        case 3: return (movimiento2==2); break;
        default: return -1;
    }
}

void acotar_camino() {
  for(int n=0; n<indice; n++) {
    for(int i=0; i<(indice-1); i++) {
      if(indice!=-1) {
        int j=i+1;
        bool encontrado=false;
        while(!encontrado && j<indice) {
          if(orientacion[j]==-1) {
            ++j;
          }
          else {
            encontrado=true;
            if(opuestos(orientacion[i], orientacion[j])) {
              orientacion[i]=-1;
              orientacion[j]=-1;
            } 
          }
        } 
      }
    }
  }
}


void loop(){
    //comment the loop to start without signal
    while(start==0){
      if(Serial1.available()){
        int val = Serial1.read();
          if(val=='s'){
            start=1;
            Serial.println(start);
          }
      }
    }
    ++tiempo;
    //Read CNY values
    Value_CNY_Back_L = analogRead(CNY_Back_L);
    Value_CNY_Back_R = analogRead(CNY_Back_R);
    Value_CNY_Front = analogRead(CNY_Front);

    //Voltage calculation1
    /*Voltage_CNY_Back_L = Value_CNY_Back_L*ResolutionADC;
    Voltage_CNY_Back_R = Value_CNY_Back_R*ResolutionADC;
    Voltage_CNY_Front = Value_CNY_Front*ResolutionADC;
    Serial1.println("Valor FRENTE: "); Serial1.println(Voltage_CNY_Front);
    Serial1.println("Valor ATRAS_R: "); Serial1.println(Voltage_CNY_Back_R);
    Serial1.println("Valor ATRAS_L: "); Serial1.println(Voltage_CNY_Back_L);
    delay(500);*/
    
    if(terminar) {
      stop();
    }
    else {
      if(hasPassedBox()) {
        delay(100);
        alinear();
        ++contadorCeldas;
        if(senal_derecha || derecha) {
          senal_delante=false;
        }
        if(senal_delante) {
          ultimo_movimiento=actualizar_movimiento(ultimo_movimiento, 0);
          ultimo=0;
          if(indice<50) {
            movimientos[indice]=ultimo;
            Serial1.println("MOVIMIENTO "); Serial1.println(ultimo);
            orientacion[indice]=ultimo_movimiento;
            ++indice;
          }
        }
      }
      if(todoNegro()) {
        ++contadorCeldas;
        terminar=true;
        Serial1.println("MOVIMIENTOS: ");
        for(int i=0; i<indice; i++) {
          Serial1.println(movimientos[i]);
          Serial1.println(" ");
        }
        Serial1.println();
        Serial1.println("ORIENTACION: ");
        for(int i=0; i<indice; i++) {
          Serial1.println(orientacion[i]);
          Serial1.println(" ");
        }
        acotar_camino();
        Serial1.println();
        Serial1.println("ORIENTACION ACOTADA: ");
        for(int i=0; i<indice; i++) {
          Serial1.println(orientacion[i]);
          Serial1.println(" ");
        }
      }
      if(!parar) {
        //Ir recto
        senal_delante=true;
        intelligentForward();
        //Si encuentra un hueco a la derecha cuando encuentra una línea atrás, girar a la derecha
        double distR=getSharpR();
        double distL=getSharpL();
        double topeR=12;
        double topeL=12;
        if((distR>topeR || distR==0) && lineFoundBack()) {
          senal_derecha=true;
          senal_delante=false;
        }
        if(senal_derecha && lineFoundFront()) {
          derecha=true;
          parar=true;
          senal_delante=false;
        }
        if(getUltrasonic()<=5 && tiempo>50) {
          parar=true;
          derecha=false;
          senal_derecha=false;
          senal_delante=false;
        }
      }
      else {
        if(derecha) {
          //Girar a la derecha
          ultimo_movimiento=actualizar_movimiento(ultimo_movimiento, 3);
          ultimo=3;
          if(indice<50) {
            movimientos[indice]=ultimo;
            orientacion[indice]=ultimo_movimiento;
            ++indice;
          }
          //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
          alinear();
          intelligentForward(255);
          delay(500);
          stop();
          rotarDerecha();
          parar=false;
          derecha=false; 
          senal_derecha=false;
          senal_delante=false;
        }
        else {
          //alinear();
          while(getUltrasonic()>6) {
            intelligentForward();
          }
          stop();
          //
          alinear();
          intelligentForward(255);
          delay(500);
          stop();
          //
          //Analizar hacia qué lado girar
          double distR=getSharpR();
          double distL=getSharpL();
          double topeR=12;
          double topeL=12;
          if(distR>topeR || distR==0 || distL>topeL || distL==0) {
            if(distR>topeR || distR==0) {
              //Girar a la derecha
              ultimo_movimiento=actualizar_movimiento(ultimo_movimiento, 3);
              ultimo=3;
              if(indice<50) {
                movimientos[indice]=ultimo;
                orientacion[indice]=ultimo_movimiento;
                ++indice;
              }
              //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
              delay(100);
              rotarDerecha();
              parar=false;
              senal_delante=false;
            }
            if(!(distR>topeR || distR==0) && (distL>topeL || distL==0)) {
              //Girar a la izquierda
              ultimo_movimiento=actualizar_movimiento(ultimo_movimiento, 2);
              ultimo=2;
              if(indice<50) {
                movimientos[indice]=ultimo;
                orientacion[indice]=ultimo_movimiento;
                ++indice;
              }
              //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
              delay(100);
              rotarIzquierda();
              parar=false;
              senal_delante=false;
            }
          }
          else {
            //Dar media vuelta
            ultimo_movimiento=actualizar_movimiento(ultimo_movimiento, 1);
            ultimo=1;
            if(indice<50) {
              movimientos[indice]=ultimo;
              orientacion[indice]=ultimo_movimiento;
              ++indice;
            }
            //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
            delay(100);
            rotar180();
            parar=false;
            senal_delante=false;
          }
        }
      } 
    }
}
