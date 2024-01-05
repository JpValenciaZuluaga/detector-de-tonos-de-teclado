#include <Servo.h>
#include <PDM.h>

#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit
uint16_t pdmDataBuffer[pdmDataBufferSize];
int16_t *x;
uint32_t PDMClk;
uint32_t MClkDiv;
int16_t *pi16PDMData;
float y[pdmDataBufferSize];
float y2[pdmDataBufferSize];
float y3[pdmDataBufferSize];
float y4[pdmDataBufferSize];
float y5[pdmDataBufferSize];
float y6[pdmDataBufferSize];
float y7[pdmDataBufferSize];
float y8[pdmDataBufferSize];
Servo myservo;  //creamos un objeto servo 
int angulo;
//Ancho de banda 1 
//852 Hz
float b1[] = {1.06316556e-05, 2.26803573e-11, -1.06316329e-05};     //{ 0.00170553889,0.00000000363847908,-0.00170553525}; prueba funcional
float a1[] = {1.0, -1.98695591, 0.99999575};                                   //{ 1.0,-1.82980289, 0.99999591};
//Ancho de banda 1 
//1336 Hz
float b2[] ={1.05810289e-05, 2.25723560e-11, -1.05810063e-05};                   //{ 0.00198136123,0.000000000422689946,-0.00198135701};
float a2[] ={1.0, -1.96797047, 0.99999577};
//Ancho de banda 2
//697 Hz
float b3[] ={1.06433032e-05,  2.27052050e-11, -1.06432805e-05}; 
float a3[] ={ 1,    -1.99132386,  0.99999574};
//1447
float b4[] ={1.05664296e-05 , 2.25412116e-11, -1.05664071e-05}; 
float a4[] ={1.0,         -1.96249563 , 0.99999577};
//Ancho de banda 3
//697 Hz
float b5[] ={1.06433032e-05,  2.27052050e-11, -1.06432805e-05}; 
float a5[] ={ 1,    -1.99132386,  0.99999574};
//1209
float b6[] ={1.05965532e-05 , 2.26054739e-11, -1.05965306e-05}; 
float a6[] ={ 1.0,     -1.97379224,  0.99999576};
//Ancho de banda 4
//1447 Hz
float b7[] ={1.05664296e-05 , 2.25412116e-11, -1.05664071e-05}; 
float a7[] ={1.0,         -1.96249563 , 0.99999577};
//770
float b8[] ={1.06380477e-05 , 2.26939935e-11 ,-1.06380250e-05}; 
float a8[] ={ 1.0,         -1.989353,    0.99999574};

float Ey,Ex,r,Ey2,r2,Ey3,r3,Ey4,r4,Ey5,r5,Ey6,r6,Ey7,r7,Ey8,r8;
int Sx,Sy,Sz,Sw,Sj,Sp,Sd,Si;

uint32_t sampleFreq;
const int analogOutPin = 8;
const int analogOutPin2 = 10;
int i = 0;
// Instanciando PDM
AP3_PDM myPDM;

void setup() {
  pinMode(7,OUTPUT);
  Serial.begin(115200);
  myservo.attach(A3);
  myPDM.begin(); delay(100);
  PDMClk = 6000000;
   MClkDiv = 1;
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));
  
}

void loop() {
  if (myPDM.available())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);
    x = (int16_t *)pdmDataBuffer;
    y[0]=0;y[1]=0;
    y2[0]=0;y2[1]=0;
    y3[0]=0;y3[1]=0;
    y4[0]=0;y4[1]=0;
    y5[0]=0;y5[1]=0;
    y6[0]=0;y6[1]=0;
    y7[0]=0;y7[1]=0;
    y8[0]=0;y8[1]=0;
    
    Ex=0;Ey=0;Ey2=0;Ey3=0;Ey4=0;Ey5=0;Ey6=0;Ey7=0;Ey8=0;
    for(int i =2; i<pdmDataBufferSize;i++)
    {
      y[i]=b1[0]*x[i]+b1[1]*x[i-1]+b1[2]*x[i-2]-(a1[1]*y[i-1]+a1[2]*y[i-2]);
      Ex= Ex+(x[i])*(x[i]);
      Ey= Ey+(y[i])*(y[i]);
      y2[i]=b2[0]*x[i]+b2[1]*x[i-1]+b2[2]*x[i-2]-(a2[1]*y2[i-1]+a2[2]*y2[i-2]);
      Ey2= Ey2+(y2[i])*(y2[i]);
      y3[i]=b3[0]*x[i]+b3[1]*x[i-1]+b3[2]*x[i-2]-(a3[1]*y3[i-1]+a3[2]*y3[i-2]);
      Ey3= Ey3+(y3[i])*(y3[i]);
      y4[i]=b4[0]*x[i]+b4[1]*x[i-1]+b4[2]*x[i-2]-(a4[1]*y[i-1]+a4[2]*y[i-2]);
      Ey4= Ey4+(y4[i])*(y4[i]);
      y5[i]=b5[0]*x[i]+b5[1]*x[i-1]+b5[2]*x[i-2]-(a5[1]*y5[i-1]+a5[2]*y5[i-2]);
      Ey5= Ey5+(y5[i])*(y5[i]);
      y6[i]=b6[0]*x[i]+b6[1]*x[i-1]+b6[2]*x[i-2]-(a6[1]*y6[i-1]+a6[2]*y6[i-2]);
      Ey6= Ey6+(y6[i])*(y6[i]);
      y7[i]=b7[0]*x[i]+b7[1]*x[i-1]+b7[2]*x[i-2]-(a7[1]*y[i-1]+a7[2]*y[i-2]);
      Ey7= Ey7+(y7[i])*(y7[i]);
      y8[i]=b8[0]*x[i]+b8[1]*x[i-1]+b8[2]*x[i-2]-(a8[1]*y8[i-1]+a8[2]*y8[i-2]);
      Ey8= Ey8+(y8[i])*(y8[i]);
    }
  // Reducción de amplitud de la energía
    Ex=Ex/1000000;
    //para Ancho de banda 1 852 Hz
    Ey=Ey/100;  
    //para Ancho de banda 1 1336 Hz
    Ey2=Ey2/100; 
    //para Ancho de banda 2 697 Hz
    Ey3=Ey3/100;
    //para Ancho de banda 2 1447 Hz
    Ey4=Ey4/100;
    //para Ancho de banda 3 697 Hz
    Ey5=Ey5/100;
    //para Ancho de banda 3 697 Hz
    Ey6=Ey6/100;
      //para Ancho de banda 4 1447 Hz
    Ey7=Ey7/100;
    //para Ancho de banda 4 770 Hz
    Ey8=Ey8/100;
    //Energia
       r=Ey/Ex;if(r>0.99)Sx=1;
    r2=Ey2/Ex;if(r2>0.99)Sy=1;
    r3=Ey3/Ex;if(r3>0.99)Sz=1;
    r4=Ey4/Ex;if(r4>0.99)Sw=1;
    r5=Ey5/Ex;if(r5>0.99)Sj=1;
    r6=Ey6/Ex;if(r6>0.99)Sp=1;
    r7=Ey7/Ex;if(r7>0.99)Sd=1;
    r8=Ey8/Ex;if(r8>0.07)Si=1;
    delay(3);
    
    
    delay(3);
    
    //Umbrales de Energia
    if(Sx==1  && Sy == 1)
      
    { 
      //delay(100);
      if( Sz==1 && Sw ==1 ) //
      { //delay(100);
        if(Sj==1 && Sp ==1 )
      { //delay(100);
         if(Sd==1 && Si ==1 )
      {
      digitalWrite(19,HIGH);
      //analogWrite(7,35);
      //delay(100);
      //analogWrite(7,105);
      angulo= 0;
      myservo.write(angulo);
      //Serial.print("ángulo:  ");
      //Serial.println(angulo);
      delay(100);  
      
      angulo= 90;
      myservo.write(angulo);
      //Serial.print("ángulo:  ");
      //Serial.println(angulo);
       delay(2000); 
      Sx=0;Sy=0;Sz=0;Sw=0;Sj=0;Sp=0;Sd=0;Si=0;
      Serial.print("Abierto");
      angulo= 0;
      myservo.write(angulo);
      
      }
     
      }
      
      }
      
    }
    else{digitalWrite(19,LOW);}
    Serial.print("Relacion");
    Serial.print(r);
    //Serial.print(',');
   // Serial.print(r);
    Serial.print(',');
    Serial.print(r2);
    Serial.print(',');
    Serial.print(r3); 
    Serial.print(',');
    Serial.print(r4);
    Serial.print(',');
    Serial.print(r6);//Serial.print("Energia");
    Serial.print(',');
    Serial.print(r7);
    Serial.print(',');
    Serial.print(r8);//Serial.print(Ex);
    //Serial.print(',');
    //Serial.print(r8);
    //Serial.print(',');
    //Serial.print(Ey);
    Serial.println();
    //delay(500);
   //Sx=0;Sy=0;Sz=0;Sw=0;Sj=0;Sp=0;Sd=0;Si=0;
  }  
  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
}
