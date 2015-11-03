
#include "quaternion.h"
#include "math.h"

// ��Ԫ��ֱ��
void quaternionMult(Quaternion* __left, Quaternion* __right,Quaternion*result )
{ 
              Quaternion _left = *__left ;
              Quaternion _right = *__right ;
              Quaternion* left = &_left ;
              Quaternion* right = &_right ;
  
              float d1, d2, d3, d4; 
  
              d1 =  left->w * right->w; 
              d2 = -left->x * right->x; 
              d3 = -left->y * right->y; 
              d4 = -left->z * right->z; 
              result->w = d1+ d2+ d3+ d4; 
  
              d1 =  left->w * right->x; 
              d2 =  right->w * left->x; 
              d3 =  left->y * right->z; 
              d4 = -left->z * right->y; 
              result->x =  d1+ d2+ d3+ d4; 
  
              d1 =  left->w * right->y; 
              d2 =  right->w * left->y; 
              d3 =  left->z * right->x; 
              d4 = -left->x * right->z; 
              result->y =  d1+ d2+ d3+ d4; 
  
              d1 =  left->w * right->z; 
              d2 =  right->w * left->z; 
              d3 =  left->x * right->y; 
              d4 = -left->y * right->x; 
              result->z =  d1+ d2+ d3+ d4; 
              
} 
  
//// ������ת��Ԫ��
void MakeRotationalQuaternion(float radian, float AxisX, float AxisY, float AxisZ,Quaternion*result) 
{ 
              float norm; 
              float ccc, sss; 
              
              result->w = result->x = result->y = result->z = 0.0f; 
  
              norm = AxisX *  AxisX +  AxisY *  AxisY +  AxisZ *  AxisZ; 
              if(norm <= 0.0f) 
                return ; 
  
              norm = 1.0f / sqrt(norm); 
              AxisX *= norm; 
              AxisY *= norm; 
              AxisZ *= norm; 
  
              ccc = cos(0.5f * radian); 
              sss = sin(0.5f * radian); 
  
              result->w = ccc; 
              result->x = sss * AxisX; 
              result->y = sss * AxisY; 
              result->z = sss * AxisZ; 
  
              return ; 
} 
  
//// ����������Ԫ��
void PutXYZToQuaternion(float PosX, float PosY, float PosZ,Quaternion*result) 
{ 
              result->w = 0.0f; 
              result->x = PosX; 
              result->y = PosY; 
              result->z = PosZ;   
} 
  

// ��Ԫ����ŷ���ǵĻ���ת����
// ʹ��������������ʱ���밴��ͷ�ļ���ŷ���������ǵ�Լ��������ŷ���Ǻ���Ԫ����



// ŷ����ת��Ԫ��

// ����ŷ��ת��������Ԫ���ı任����:
// q = qyaw qpitch qroll
// ����
// qroll = [cos(y/2), (sin(y/2), 0, 0)];
// qpitch = [cos(q/2), (0, sin(q/2), 0)];
// qyaw = [cos(f/2), (0, 0, sin(f/2)];
void EulerToQuat( float roll, float pitch, float yaw, Quaternion* result)
{
 float cr, cp, cy, sr, sp, sy, cpcy, spsy;
 
 //��������Ԫ��ʱʹ�õ�����������ֵ
 cr = cos(roll / 2);
 cp = cos(pitch / 2);
 cy = cos(yaw / 2);

 sr = sin(roll/2);
 sp = sin(pitch/2);
 sy = sin(yaw/2);
 cpcy = cp * cy;
 spsy = sp * sy;

 //�����Щֵ,������Ԫ����������w
 result->w = cr*cpcy + sr*spsy;
 result->x = sr*cpcy - cr*spsy;
 result->y = cr*sp*cy + sr*cp*sy;
 result->z = cr*cp*sy - sr*sp*cy;
}




// ��Ԫ��תŷ����

void QuatToEuler ( Quaternion*quat , EulerAngle* angle )
{
#define PIOver2 (3.1415926535897932384626433832795f/2.0f)   
#define PI 3.1415926535897932384626433832795f

   float w = quat->w ;
   float x = quat->x ;
   float y = quat->y ;
   float z = quat->z ;

   float test = w*y - z*x ;
   if (test > 0.4999999f)
   {
    angle->yaw = 2.0f * atan2(z, w);
    if ( angle->yaw < -PI )  angle->yaw += 2.0*PI ;
    else if ( angle->yaw > PI )  angle->yaw -= 2.0*PI ;
    angle->pitch = PIOver2;
    angle->roll = 0.0f;
	return ;
   }
   if (test < -0.4999999f)
   {
    angle->yaw = 2.0f * atan2(z, w);
    if ( angle->yaw < -PI )  angle->yaw += 2.0f*PI ;
    else if ( angle->yaw > PI )  angle->yaw -= 2.0f*PI ;
    angle->pitch = -PIOver2;
    angle->roll = 0.0f;
	return ;
   }
   float sqx = x * x;
   float sqy = y * y;
   float sqz = z * z;

#define CLAMP(x , min , max) ((x) > (max) ? (max) : ((x) < (min) ? (min) : x))
   angle->yaw = atan2(2.0f * w * z + 2.0f * x * y , 1.0f - 2.0f * sqy - 2.0f * sqz);
   angle->pitch = asin( CLAMP(2.0f * test,-1.0f,1.0f));
   angle->roll = atan2(2.0f * w * x  + 2.0f * y * z, 1.0f - 2.0f * sqx - 2.0f * sqy);
}

