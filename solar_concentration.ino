// demo code to calculate mirror normal position for solar concentrator
//

// convert (alt,az) in degrees to unit vector xyz Coordinate frame: (Y North, X East, Z up)
//
void altaz_to_xyz(float alt, float az, float xyz[3]) {
  float dtr = PI / 180.0;
  *xyz++ = cos(alt * dtr) * sin(az * dtr);
  *xyz++ = cos(alt * dtr) * cos(az * dtr);
  *xyz = sin(alt * dtr);
}
//
// converte unit vector xyz (y North, x East, Z up) to (alt, az) in degrees
//
void xyz_to_altaz(float * alt, float * az, float xyz[3]) {
  float rtd = 180.0 / PI;
  float v[3], m = 0;
  // normalize input vector
  for (byte i = 0; i < 3; i++) m += xyz[i] * xyz[i];
  m = sqrt(m);
  for (byte i = 0; i < 3; i++) v[i] = xyz[i] / m;
  float t = rtd * atan2(v[0], v[1]);
  if (t < 0.0) t += 360.0; //compass wrap
  *az = t;
  *alt = rtd * asin(v[2]);
}
//
// function to calculate mirror normal altitude and azimuth, given solar and target altitude and aximuth
// angles in decimal degrees
// Cartesian coordinate convention: Y points true North, Z up, X East
//
void solar_concentrator(float SunAlt, float SunAz, float TargetAlt, float TargetAz, float * MirrorNormalAlt, float * MirrorNormalAz) {
  float Sxyz[3], Txyz[3], Nxyz[3];
  // get sun 3D coordinates
  altaz_to_xyz(SunAlt, SunAz, Sxyz);
  // get target 3D coordinates
  altaz_to_xyz(TargetAlt, TargetAz, Txyz);
  // calculate mirror normal vector
  Nxyz[0] = Sxyz[0] + Txyz[0];
  Nxyz[1] = Sxyz[1] + Txyz[1];
  Nxyz[2] = Sxyz[2] + Txyz[2];
  // calculate mirror normal alt, az
  xyz_to_altaz(MirrorNormalAlt, MirrorNormalAz, Nxyz);
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // test data, degrees
  float SunAlt = 26.44;
  float SunAz = 204.71;
  float TargetAlt = 14.83;
  float TargetAz = 168.14;
  float alt, az; //mirror normal coords
  
  solar_concentrator(SunAlt, SunAz, TargetAlt, TargetAz, &alt, &az);
  
  Serial.print(F("Sun alt, az "));
  Serial.print(SunAlt);
  Serial.print(F(" "));
  Serial.println(SunAz);
  
  Serial.print(F("Target alt, az: "));
  Serial.print(TargetAlt);
  Serial.print(F(" "));
  Serial.println(TargetAz);
  
  // next lines should print 21.63, 185.70
  Serial.print(F("Calculated mirror normal alt, az "));
  Serial.print(alt);
  Serial.print(F(" "));
  Serial.println(az);
}

void loop() {
  // put your main code here, to run repeatedly:

}
