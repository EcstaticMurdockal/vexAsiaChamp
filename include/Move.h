void Move(int left_power,int right_power);

void getEncoder();


void Move_Forward(int left_power,int right_power);
void Move_Reverse(int left_power,int right_power);

void Move_Stop_brake(void);
void Move_Stop_coast(void);
void Move_Stop_hold(void);
void toPos(double x, double y, double pow);
void turnTo(double target,double pow);

//void Roller_Up_Control(int power);
//void Roller_Intake_Control(int power);

extern float lm1_enc; // Setting up variables for motor encoders
extern float rm1_enc;
extern float lm2_enc;
extern float rm2_enc;
extern float lm3_enc;
extern float rm3_enc;

extern double hol;

extern bool alliance;
extern double target_H;