/* Se utiliza el MPU6050 con Arduino para extraer una medición del ángulo de inclinación "X" */
/* (según la referencia del MPU6050)* y enviarla por puerto serie.			     */

#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 IMUCDR;      // Creo la clase IMUCDR del tipo MPU6050.
                     // Esto supone que AD0 está a GND.
                    
int16_t ax = 0, ay = 0, az = 0;  // Creo las variables para el acelerómetro 
int16_t gx = 0, gy = 0, gz = 0;  // y para el giróscopo.

int16_t ax_cal = 0, ay_cal = 0, az_cal = 0;  // Creo las variables para el acelerómetro 
int16_t gx_cal = 0, gy_cal = 0, gz_cal = 0;  // y para el giróscopo luego de la calibración.

unsigned long last_read_time;
float last_angulox;
float last_anguloy;
float last_anguloz;
float last_Gxangulo;
float last_Gyangulo;
float last_Gzangulo;

#define N 10

#define LED_PIN 13   // Defino un LED.


bool blinkState = false;


void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE        // Habría que mirar estas declaraciones en I2Cdev.h
        Wire.begin();                                       // Configura como maestro al Arduino de I2C.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);                         // Habría que ver que es esta función.
    #endif
    
    Serial.begin(9600);                                     // Esta velocidad anda bien con 8MHz y 16 MHz.
    Serial.println("Inicializando dispositivo I2C...");
    IMUCDR.initialize();                                    // Esta función saca del estado sleep al chip.
                                                            // También setea al acelerómetro en +/- 2g y al 
                                                            // giro en +/- 250 º/s.
                                                            // Elige como fuente de reloj el X del giro, que 
                                                            // es mejor que la fuente interna por defecto.
    Serial.println("Verificando conexiones");
    Serial.println(IMUCDR.testConnection() ? "MPU6050 todo bien" : "MPU6050 hubo error");
                                                            // Esta función consulta con 0x34 si el dispositivo
                                                            // está conectado.
    //pinMode(LED_PIN, OUTPUT);
    
    calibrate_sensors();
    set_last_read_angle_data(millis(),0,0,0,0,0,0);
}

void loop()
{    
    IMUCDR.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);          // Mido los valores de acelerometro y giroscopio crudos.
    unsigned long t_now = millis();                           // Mido el tiempo
   
    float FS_SEL = 131;                                       // Si divido por este valor a Gx,Gy,Gz los pongo en º/seg.
    float Gx = (gx - gx_cal)/FS_SEL;                          // Resto los valores de calibración.
    float Gy = (gy - gy_cal)/FS_SEL;
    float Gz = (gz - gz_cal)/FS_SEL;
    
    float Ax = ax;
    float Ay = ay;
    float Az = az;
    
    float RADIANES_A_GRADOS = 180/3.14159;
    
    float anguloAy = atan(-1*Ax/sqrt(pow(Ay,2) + pow(Az,2)))*RADIANES_A_GRADOS; // En internet figura de donde sale.
    float anguloAx = atan(Ay/sqrt(pow(Ax,2) + pow(Az,2)))*RADIANES_A_GRADOS;
    float anguloAz = 0;
    
    float dt =(t_now - get_last_time())/1000.0;               // Intervalo de "integracion"
    float anguloGx = Gx*dt + get_last_angulox();              // Angulo a partir del giroscopio, integrando omega y sumando el angulo anterior total el eje.
    float anguloGy = Gy*dt + get_last_anguloy();
    float anguloGz = Gz*dt + get_last_anguloz();
        
    float unfiltered_anguloGx = Gx*dt + get_last_Gxangulo();  // Calculo los que servirán para cargar el filtro.
    float unfiltered_anguloGy = Gy*dt + get_last_Gyangulo();
    float unfiltered_anguloGz = Gz*dt + get_last_Gzangulo();    
    
    float alpha = 0.96;
    float angulox = alpha*anguloGx + (1.0 - alpha)*anguloAx;
    float anguloy = alpha*anguloGy + (1.0 - alpha)*anguloAy;
    float anguloz = anguloGz;  //Accelerometer doesn't give z-angle
    
    set_last_read_angle_data(t_now, angulox, anguloy, anguloz, unfiltered_anguloGx, unfiltered_anguloGy, unfiltered_anguloGz);
    
    Serial.println(angulox);  
    
    delay(5);

}


void calibrate_sensors()
{
    int16_t AX,AY,AZ,GX,GY,GZ; 
    IMUCDR.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Descarto el primer valor de todo.
    
    for(int i = 0; i < N; i++)           // Promedio N valores.
    {   
      IMUCDR.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      AX += ax;
      AY += ay;
      AZ += az;
      GX += gx;
      GY += gy;
      GZ += gz;
      delay(100);
    }
    AX /= N;
    AY /= N;
    AZ /= N;
    GX /= N;
    GY /= N;
    GZ /= N;
    
    ax_cal = AX;          // Guardo los valores "calibrados".
    ay_cal = AY;
    az_cal = AZ;
    gx_cal = GX;
    gy_cal = GY;
    gz_cal = GZ;
}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) 
{
  last_read_time = time;
  last_angulox = x;
  last_anguloy = y;
  last_anguloz = z;
  last_Gxangulo = x_gyro;
  last_Gyangulo = y_gyro;
  last_Gzangulo = z_gyro;
}

inline unsigned long get_last_time() {return last_read_time;}
inline float get_last_angulox() {return last_angulox;}
inline float get_last_anguloy() {return last_anguloy;}
inline float get_last_anguloz() {return last_anguloz;}
inline float get_last_Gxangulo() {return last_Gxangulo;}
inline float get_last_Gyangulo() {return last_Gyangulo;}
inline float get_last_Gzangulo() {return last_Gzangulo;}

