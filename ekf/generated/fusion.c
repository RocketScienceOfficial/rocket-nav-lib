

H_Fusion[0] = 1;
H_Fusion[1] = 1;


const float KS0 = P[0][0]*R_BARO;
const float KS1 = P[0][0]*R_GPS;
const float KS2 = 1.0F/(KS0 + KS1 + R_BARO*R_GPS);


K_Fusion[0] = KS0*KS2;
K_Fusion[1] = KS1*KS2;


