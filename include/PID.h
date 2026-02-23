void EncoderReset();
//extern double error,PID_out;
//void PID_Turn(int degree);
//void PID_Move(int left_power,int right_power,int degree,int timelimit);
void GyroMove( double power, double enc , double o);//GyroMove without pid
void pidMove(float maxPower, float target);
void PIDGMove(double enc , double o, double timeout);
void pid_until_dis(float target_dis, bool isfront, double o, int timeout);
void GMove( double power, double enc);

void Turn(float target, int timeout);
void Turn_Beta(float target);

void MoveTo(float tar_x,float tar_y,int fulltime,int min_power);

void Move_to_yellow(double enc, double pow);
void Move_to_yellow_front(double enc, double pow);
void Move_to_blue(double enc, double pow);
void Move_to_red(double enc, double pow);


int sgn(double d);
void turn(int turnpwm,int forwardpwm);
void forward_(double leftpower,double rightpower);
void EncoderReset(void);
void Enc_pos();
void Pos_Motor();

extern double X;
extern double Y;
extern double H;
extern double Xi;
extern double Yi;
extern double xd;//desired x
extern double yd;//desired y
extern double hd;//desired h
extern double ax;
extern double ay;
extern double vx;
extern double vy;
extern double vxi;
extern double vyi;
extern double dt;
extern float L_dis;
extern float R_dis;
extern float front_dis;
extern float back_dis;

extern double drive_enc;


extern double sgnAx;
extern double sgnAy;
extern double lastvx;
extern double lastvy;

extern bool is_executing_movements;
