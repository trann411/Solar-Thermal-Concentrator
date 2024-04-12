void setup() {
  // put your setup code here, to run once:
  char *token;
  char *mystring = "test 94";
  const char *delimiter = " ";

  Serial.begin(115200);

  token = strtok(mystring, delimiter);

  delay(5000);

  Serial.println(token);
  
  /*
  while (token != NULL){
    Serial.println(token);
    token = strtok(NULL, delimiter);
  }
  */

}

void loop() {
  // put your main code here, to run repeatedly:

}
