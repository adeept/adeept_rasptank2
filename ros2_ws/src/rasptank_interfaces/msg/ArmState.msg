# Estado del brazo. Los servos no dan realimentación: es una estimación.
std_msgs/Header header
string pose       # última postura con nombre pedida ("" si se movió por articulaciones)
bool moving       # true mientras los servos van hacia la consigna
bool stopped      # parada de emergencia activa
