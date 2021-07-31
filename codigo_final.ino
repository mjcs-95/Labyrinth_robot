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
const float ResolutionADC=0.00488f; //4.88mV
unsigned long time_bounce;
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

void Stop() {
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
}

void StopRight() {
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin2RightMotor, 0);
}

void StopLeft() {
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1Left_Motor, 0);
}

void Forward(int AnalogValue) {
    Stop();
    //Suponiendo una diferencia rotacional entre ruedas del 30%
    diferencia=0.2*AnalogValue;
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, AnalogValue-diferencia);
    analogWrite(pin2RightMotor, AnalogValue);
}

void TackRight() {
    /*diferencia=0.2*AnalogValue;*/
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 128);
    analogWrite(pin2RightMotor, 255);
}

void TackLeft() {
    /*diferencia=0.2*AnalogValue;*/
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 0);
    analogWrite(pin1Left_Motor, 255);
    analogWrite(pin2RightMotor, 128);
}

void Back(int AnalogValue) {
    Stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, AnalogValue);
    analogWrite(pin1RightMotor, AnalogValue);
}

void BackLeft() {
    Stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, 255);
    analogWrite(pin1RightMotor, 0);
}

void BackRight() {
    Stop();
    analogWrite(pin1Left_Motor, 0);
    analogWrite(pin2RightMotor, 0);
    analogWrite(pin2Left_Motor, 0);
    analogWrite(pin1RightMotor, 255);
}

float VoltageFromAnalogRead(int pin){
    return analogRead(pin)*ResolutionADC;
}

void UpdateCNYVoltage(){
    Voltage_CNY_Back_L = VoltageFromAnalogRead(CNY_Back_L);
    Voltage_CNY_Back_R = VoltageFromAnalogRead(CNY_Back_R);
    Voltage_CNY_Front = VoltageFromAnalogRead(CNY_Front);
}

//Previously the Voltage required in all CNYs was 3.4
bool LineFoundFront() {
    UpdateCNYVoltage();
    return (Voltage_CNY_Front>3.4); /*&& (Voltage_CNY_Front-min(Voltage_CNY_Back_L, Voltage_CNY_Back_R))>1*/
}

bool LineFoundBack() {
    UpdateCNYVoltage();
    return (Voltage_CNY_Back_L>3.7 || Voltage_CNY_Back_R>3.7); /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/
}

bool LineFoundBack_L() {
    UpdateCNYVoltage();
    return (Voltage_CNY_Back_L>3.7); /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/ 
}

bool LineFoundBack_R() {
    UpdateCNYVoltage();
    return (Voltage_CNY_Back_R>3.7); /*&& (max(Voltage_CNY_Back_L, Voltage_CNY_Back_R)-Voltage_CNY_Front)>1*/ 
}

bool TodoNegro() {
    UpdateCNYVoltage();
    return (Voltage_CNY_Front>3.4 && Voltage_CNY_Back_L>3.7 && Voltage_CNY_Back_R>3.7);
}

bool noLineFound() {
    return !LineFoundFront() && !LineFoundBack();
}

/* Return the distance(in cm) of a Sharp sensor based on its voltage*/
double GetDistanceFromSharpVoltage(double valor) {  
    if(valor<0.5 || valor>2.7) {
        return 0;
    } else {
        double centimetros=valor/13.05;
        centimetros=(1-centimetros*0.42)/centimetros;
        return centimetros;
    }
}

double GetRightSharpDistance() {
    return GetDistanceFromSharpVoltage( VoltageFromAnalogRead(SharpR) );
}

double GetLeftSharpDistance() {
    return GetDistanceFromSharpVoltage( VoltageFromAnalogRead(SharpL) );
}

/*Return the distance from Ultrasonic Sensor*/
double GetUltrasonicDistance() {
    digitalWrite(Trigger, HIGH); //send pulse for 10 us
    delayMicroseconds(10);
    digitalWrite(Trigger, LOW);
    time_bounce=pulseIn(Echo, HIGH);
    return 0.017 * time_bounce; //Distance formula
}

void setup(){
    pinMode(pin1Left_Motor, OUTPUT);
    pinMode(pin2Left_Motor, OUTPUT);
    pinMode(pin1RightMotor, OUTPUT);
    pinMode(pin2RightMotor, OUTPUT);
    pinMode(Trigger, OUTPUT); 
    digitalWrite(Trigger, LOW);
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

bool HasPassedBox() {
    int estado_anterior=estado;
    //Actualizar estado
    switch(estado) {
        case 1: 
            if ( LineFoundFront() ) estado=2; break;
        case 2: 
            if( LineFoundBack() && !LineFoundFront() ) estado=3; break;
        case 3: 
            if( !LineFoundBack() ) estado=1;
    }
    //Fin actualizar estado
    if(estado_anterior==3 && estado==1) { 
        Serial1.println('m'); return true;
    } else {
        return false;
    }
}

void balanceLeft() {  /* Mantener el robot con la referencia de la pared de la izquierda */
    double distL=GetLeftSharpDistance();
    if(distL<10) {   //Valía 6
        //Serial1.println("VIRANDO A LA DERECHA");
        TackRight();
        delay(50);
        Stop();
    } else {
        //Serial1.println("VIRANDO A LA IZQUIERDA");
        TackLeft();
        delay(50);
        Stop();
    }
}

void balanceRight() {  /* Mantener el robot con la referencia de la pared de la derecha */
    double distR=GetRightSharpDistance();
    if(distR<10) {   //Valía 6
        TackLeft();
        delay(50);
        Stop();
    } else {
        TackRight();
        delay(50);
        Stop();
    }
}

void IntelligentForward(int potencia=150) {
    double distR=GetRightSharpDistance();
    double distL=GetLeftSharpDistance();
    double diferencia=distR-distL;
    double umbralp=2;
    double umbraln=-umbralp;
    double topeR=12;
    double topeL=12;
    
    if(distR>topeR || distR==0 || distL>topeL || distL==0) {
        if((distR>topeR || distR==0) && (distL>topeL || distL==0)) {
            Forward(potencia);
        }
        if((distR>topeR || distR==0) && !(distL>topeL || distL==0)) {
            //Serial1.println("BALANCEANDO A LA IZQUIERDA");
            balanceLeft();
            Forward(potencia);
        }
        if(!(distR>topeR || distR==0) && (distL>topeL || distL==0)) {
            //Serial1.println("BALANCEANDO A LA DERECHA");
            balanceRight();
            Forward(potencia);
        }
    } else {
        if(diferencia>umbralp) {
            TackRight();
            delay(50);
            Stop();
        } else {
            if(diferencia<umbraln) {
                TackLeft();
                delay(50);
                Stop();
            } else {
                Forward(potencia);
            }
        }
    }
}

void Alinear() {
    Stop();
    bool alineado=false;
    while(!alineado) {
        if(!LineFoundBack()) {
            //Ir atrás
            Back(255);
        } else {
            if(LineFoundBack_R() && LineFoundBack_L()) {
                //Ya está alineado
                alineado=true;
                Stop();
            } else {
                if(LineFoundBack_L()) {
                    //Mover culo atrás a la izquierda
                    Serial1.println("YENDO ATRAS POR LA IZQUIERDA");            
                    //Voltage calculation1
                    Voltage_CNY_Back_L = VoltageFromAnalogRead(CNY_Back_L);
                    Serial1.println("Valor ATRAS_L: "); Serial1.println(Voltage_CNY_Back_L);
                    BackLeft();
                }
                if(LineFoundBack_R()) {
                    //Mover culo atrás a la derecha
                    BackRight();
                }
            }
        }
    }
}

bool RevolucionesPin2(int transiciones) {
    transicionPin2=0;
    while(true) {
        //Serial1.println("VALOR DEL PIN 2: "); Serial1.println(digitalRead(2));
        int Pin2=digitalRead(2);
        if((antPin2==1 && Pin2==0) || (antPin2==0 && Pin2==1)) {
            if(antPin2==1 && Pin2==0) {
                antPin2=0; 
            } else {
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

bool RevolucionesPin3(int transiciones) {
    transicionPin3=0;
    while(true) {
        //Serial1.println("VALOR DEL PIN 3: "); Serial1.println(digitalRead(3));
        int Pin3=digitalRead(3);
        if((antPin3==1 && Pin3==0) || (antPin3==0 && Pin3==1)) {
            if(antPin3==1 && Pin3==0) {
                antPin3=0; 
            } else {
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

void RotarDerecha(int iteraciones=10) {
    Serial1.println('r');
    for(int a=0; a<iteraciones; a++) {
        analogWrite(pin2Left_Motor, 150);
        while(!RevolucionesPin3(1)) {}
        analogWrite(pin2RightMotor, 150);
        while(!RevolucionesPin2(1)) {}
    }
    Stop();
}

void RotarIzquierda(int iteraciones=10) {
    Serial1.println('l');
    for(int a=0; a<iteraciones; a++) {
        analogWrite(pin1Left_Motor, 150);
        while(!RevolucionesPin3(1)) {}
        analogWrite(pin1RightMotor, 150);
        while(!RevolucionesPin2(1)) {}
    }
    Stop();
}

void Atras(int iteraciones=7) {
    for(int a=0; a<iteraciones; a++) {
        bool pararIzquierda=false;
        bool pararDerecha=false;
        if(pararIzquierda) {
            StopLeft();
        }
        else {
            //Mover rueda izquierda durante x revoluciones
            analogWrite(pin2Left_Motor, 255);
            if(RevolucionesPin3(1)) {
                pararIzquierda=true;
            }
        }
        if(pararDerecha) {
            StopRight();
        }
        else {
            //Mover rueda derecha durante x revoluciones
            analogWrite(pin1RightMotor, 255);
            if(RevolucionesPin2(1)) {
                pararDerecha=true;
            }
        }
    }
    Stop();
}

void Rotar180() {
    //Serial1.println('b');
    RotarIzquierda(6);
    Atras();
    RotarIzquierda(17);
    estado=2;
}

int ActualizarMovimiento(int ultimo_movimiento, int movimiento) {
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

bool Opuestos(int movimiento1, int movimiento2) {
    switch(movimiento1) {
        case 0: return (movimiento2==1);
        case 1: return (movimiento2==0);
        case 2: return (movimiento2==3);
        case 3: return (movimiento2==2);
        default: return false;
    }
}

void AcotarCamino() {
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
                        if(Opuestos(orientacion[i], orientacion[j])) {
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
    /* 
    UpdateCNYVoltage();
    Serial1.println("Valor FRENTE: "); Serial1.println(Voltage_CNY_Front);
    Serial1.println("Valor ATRAS_R: "); Serial1.println(Voltage_CNY_Back_R);
    Serial1.println("Valor ATRAS_L: "); Serial1.println(Voltage_CNY_Back_L);
    delay(500); 
    */
    if(terminar) {
        Stop();
    } else {
        if(HasPassedBox()) {
            delay(100);
            Alinear();
            ++contadorCeldas;
            if(senal_derecha || derecha) {
                senal_delante=false;
            }
            if(senal_delante) {
                ultimo_movimiento=ActualizarMovimiento(ultimo_movimiento, 0);
                ultimo=0;
                if(indice<50) {
                    movimientos[indice]=ultimo;
                    Serial1.println("MOVIMIENTO "); Serial1.println(ultimo);
                    orientacion[indice]=ultimo_movimiento;
                    ++indice;
                }
            }
        }
        if(TodoNegro()) {
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
            AcotarCamino();
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
            IntelligentForward();
            //Si encuentra un hueco a la derecha cuando encuentra una línea atrás, girar a la derecha
            double distR=GetRightSharpDistance();
            double distL=GetLeftSharpDistance();
            double topeR=12;
            double topeL=12;
            if((distR>topeR || distR==0) && LineFoundBack()) {
                senal_derecha=true;
                senal_delante=false;
            }
            if(senal_derecha && LineFoundFront()) {
                derecha=true;
                parar=true;
                senal_delante=false;
            }
            if(GetUltrasonicDistance()<=5 && tiempo>50) {
                parar=true;
                derecha=false;
                senal_derecha=false;
                senal_delante=false;
            }
        } else {
            if(derecha) {
                //Girar a la derecha
                ultimo_movimiento=ActualizarMovimiento(ultimo_movimiento, 3);
                ultimo=3;
                if(indice<50) {
                    movimientos[indice]=ultimo;
                    orientacion[indice]=ultimo_movimiento;
                    ++indice;
                }
                //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
                Alinear();
                IntelligentForward(255);
                delay(500);
                Stop();
                RotarDerecha();
                parar=false;
                derecha=false; 
                senal_derecha=false;
                senal_delante=false;
            } else {
                //Alinear();
                while(GetUltrasonicDistance()>6) {
                    IntelligentForward();
                }
                Stop();
                //
                Alinear();
                IntelligentForward(255);
                delay(500);
                Stop();
                //
                //Analizar hacia qué lado girar
                double distR=GetRightSharpDistance();
                double distL=GetLeftSharpDistance();
                double topeR=12;
                double topeL=12;
                if(distR>topeR || distR==0 || distL>topeL || distL==0) {
                    if(distR>topeR || distR==0) {
                        //Girar a la derecha
                        ultimo_movimiento=ActualizarMovimiento(ultimo_movimiento, 3);
                        ultimo=3;
                        if(indice<50) {
                            movimientos[indice]=ultimo;
                            orientacion[indice]=ultimo_movimiento;
                            ++indice;
                        }
                        //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
                        delay(100);
                        RotarDerecha();
                        parar=false;
                        senal_delante=false;
                    }
                    if(!(distR>topeR || distR==0) && (distL>topeL || distL==0)) {
                        //Girar a la izquierda
                        ultimo_movimiento=ActualizarMovimiento(ultimo_movimiento, 2);
                        ultimo=2;
                        if(indice<50) {
                            movimientos[indice]=ultimo;
                            orientacion[indice]=ultimo_movimiento;
                            ++indice;
                        }
                        //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
                        delay(100);
                        RotarIzquierda();
                        parar=false;
                        senal_delante=false;
                    }
                } else {
                    //Dar media vuelta
                    ultimo_movimiento=ActualizarMovimiento(ultimo_movimiento, 1);
                    ultimo=1;
                    if(indice<50) {
                        movimientos[indice]=ultimo;
                        orientacion[indice]=ultimo_movimiento;
                        ++indice;
                    }
                    //Serial1.println("MOVIMIENTO REGISTRADO: "); Serial1.println(ultimo);
                    delay(100);
                    Rotar180();
                    parar=false;
                    senal_delante=false;
                }
            }
        } 
    }
}
