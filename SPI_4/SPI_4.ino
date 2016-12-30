
/*Tutorial SPI Receptor Slave
 www.electroensaimada.com
 Autor:Toni Ruiz Sastre
 */
 
 #include <SPI.h>
char buf [150];
volatile byte pos;
volatile boolean recibido;

void setup (void)
{
  Serial.begin (9600);
  pinMode(MISO, OUTPUT);//Configuramos el Miso como salida ya que el Master recibira por aqui.
  Serial.println("HELLO WORLD");
  SPCR |= _BV(SPE);//Activamos el modo Slave
  pos = 0;   // Posicion 0 de los bytes recibidos
  recibido = false; //Inicializamos que no hemos recibido.
  SPI.attachInterrupt();//Activamos la interrupcion del SPI
}

// Interrupcion SPI
ISR (SPI_STC_vect)
{
  byte c = SPDR;  //Obtenemos el byte
  
    buf [pos++] = c;
    if (c == '\n')
    recibido = true;
  
}

void loop (void)
{
  if (recibido)
  {
    buf [pos] = 0;  
    Serial.println (buf);
    pos = 0;
    recibido = false;
  } 
}
