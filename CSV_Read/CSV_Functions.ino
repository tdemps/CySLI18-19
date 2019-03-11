/*
   Read a file one field at a time.
   file - File to read.
   str - Character array for the field.
   size - Size of str array.
   delim - csv delimiter.
   return - negative value for failure.
            delimiter, '\n' or zero(EOF) for success.
*/
int csvReadText(File* file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadInt32(File* file, int32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtol(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadInt16(File* file, int16_t* num, char delim) {
  int32_t tmp;
  int rtn = csvReadInt32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp < INT_MIN || tmp > INT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadUint32(File* file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadUint16(File* file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
int csvReadDouble(File* file, double* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtod(buf, &ptr);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
//------------------------------------------------------------------------------
int csvReadFloat(File* file, float* num, char delim) {
  double tmp;
  int rtn = csvReadDouble(file, &tmp, delim);
  if (rtn < 0)return rtn;
  // could test for too large.
  *num = tmp;
  return rtn;
}
//------------------------------------------------------------------------------
void csvSetup() {
  // Remove existing file.
//    SD.remove("READTEST.TXT");

  // Write test data.   (Disabled for my test with real data)
//    file.print(F(
//  "36,23.20,20.70,57.60,79.50,01:08:14,23.06.16\r\n"
//  "37,23.21,20.71,57.61,79.51,02:08:14,23.07.16\r\n"
//  ));

//  // Must be dim 9 to allow for zero byte.

  while (csv.available()) {
  if (csvReadUint32(&csv, &time, delim) != delim
        || csvReadDouble(&csv, &altitude, delim) != delim
        || csvReadDouble(&csv, &accX, delim) != delim
        || csvReadDouble(&csv, &accY, delim) != delim
        || csvReadDouble(&csv, &accZ, delim) != '\n'){
    //    || csvReadText(&file, dateS, sizeof(dateS), delim) != '\n') {
      Serial.println("read error");
      int ch;
      delay(100);
      int nr = 0;
      // print part of file after error.
      while ((ch = csv.read()) > 0 && nr++ < 100) {
        delay(10);
        Serial.write(ch);
      }
      break;
    }
    Serial.print(time);
    Serial.print(delim);
    Serial.print(altitude);
    Serial.print(delim);
    Serial.print(accX);
    Serial.print(delim);
    Serial.print(accY);
    Serial.print(delim);
    Serial.println(accZ);
  }
}
