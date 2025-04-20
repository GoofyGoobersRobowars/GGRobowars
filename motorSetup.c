void motorSetup(){
  // Back Left Setup
  pinMode(FBL, OUTPUT);
  pinMode(BBL, OUTPUT);
  pinMOde(PWM_BL, OUTPUT);

  // Back Right Setup
  pinMode(FBR, OUTPUT);
  pinMode(BBR, OUTPUT);
  pinMode(PWM_BR, OUTPUT);

  // Front Right Setup
  pinMode(FFR, OUTPUT);
  pinMode(BFR, OUTPUT);
  pinMOde(PWM_FR, OUTPUT);

  // Front Left Setup
  pinMode(FFL, OUTPUT);
  pinMode(BFL, OUTPUT);
  pinMode(PWM_FL, OUTPUT);

  // STBY Setup
  pinMode(STBY1, OUTPUT);
  pinMode(STBY2, OUTPUT);
}
