#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_


#include <Arduino.h>




class Quaternion {
    public:
    
        float w;
        float x;
        float y;
        float z;
        
        Quaternion() {
            w = 0.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }

        Quaternion(Quaternion axis, float angle) {

            angle /= 2.0f;
            float sa = sinf(angle);
            
            w = cosf(angle);
            x = axis.x*sa;
            y = axis.y*sa;
            z = axis.z*sa;

        }
        
        Quaternion(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion(float nx, float ny, float nz) {
            w = 0.0f;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion getProductQuat(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float getMagnitude() {

            float m = w*w + x*x + y*y + z*z;

            m = sqrtf(m);
            
            if (m == m) {
                return m;
            }

            return 0.0f;
        }
        
        void normalize(bool sign = false) {
            float m = getMagnitude();

            if (!sign && m == 1.0f) return;

            if (m > 0.00001f) {

                if (w < 0.0f && sign) {
                    m = -m;
                }

                w /= m;
                x /= m;
                y /= m;
                z /= m;

                return;

            } else if (w > 0.0f || x > 0.0f || y > 0.0f || z > 0.0f) {

                m = 100000;

                w *= m;
                x *= m;
                y *= m;
                z *= m;

                return;

            }

            w = 0.0f;
            x = y = z = 0.0f;

        }
        
        Quaternion getNormalized(bool sign = false) {
            Quaternion r(w, x, y, z);
            r.normalize(sign);
            return r;
        }

        float getAngleTo(Quaternion b) {
            float phi = *this*b;

            phi = acosf(phi/(this->getMagnitude()*b.getMagnitude()));

            if (phi == phi) {
                return phi;
            }

            return 0.0f;
        }

        Quaternion operator + (Quaternion b) {
            return Quaternion(w + b.w, x + b.x, y + b.y, z + b.z);
        }

        Quaternion operator - (Quaternion b) {
            return Quaternion(w - b.w, x - b.x, y - b.y, z - b.z);
        }

        Quaternion operator - (void) {
            return Quaternion(-w, -x, -y, -z);
        }

        Quaternion operator * (float c) {
            return Quaternion(w*c, x*c, y*c, z*c);
        }

        Quaternion operator / (float c) {
            return Quaternion(w/c, x/c, y/c, z/c);
        }

        void operator *= (float c) {
            w *= c;
            x *= c;
            y *= c;
            z *= c;
        }

        void operator += (Quaternion b) {
            w += b.w;
            x += b.x;
            y += b.y;
            z += b.z;
        }

        float operator * (Quaternion b) {
            return w*b.w + x*b.x + y*b.y + z*b.z;
        }

        Quaternion getCross(Quaternion b) {
            return Quaternion(
                0.0f,
                y*b.z-z*b.y,
                z*b.x-x*b.z,
                x*b.y-y*b.x
            );
        } 

};



class Position{

    
    public:

        double theta, phi, height;

        const double polarRadius = 6356752.3;
        const double equatorialRadius = 6378137.0;

        Position(double lat = 0, double lon = 0, double MSLHeight = 0) {
            theta = lat;
            phi = lon;
            height = MSLHeight;
        };

        void setPosFromGPS(double lat, double lon, double MSL) {
            theta = lat;
            phi = lon;
            height = MSL;
        }

        void changePosFromSpeed(Quaternion v, double time) {
            
            v.x = v.x*time;
            v.y = v.y*time;
            v.z = v.z*time;

            this->changePosFromQuat(v);

        }

        void changePosFromQuat(Quaternion v) {
            
            height -= double(v.z); //negative for NED coordinate system.

            double r = getRadius() + height;

            double thetaSpd = double(v.x)/(r);
            theta -= thetaSpd;

            double phiSpd = double(v.y)/(r*cos(theta));
            phi -= phiSpd;

        }

        double getLat() {return theta*180.0/pi;}
        double getLon() {return phi*180.0/pi;}
        double getHeightMSL() {return height;}

        double getLatRad() {return theta;}
        double getLonRad() {return phi;}

        void setHeightMSL(float HeightMSL) {

            height = HeightMSL;

        }

        Quaternion getDistanceTo(Position b) {

            Quaternion v;

            v.x = float((b.getLatRad() - theta)*polarRadius);
            v.y = -float((b.getLonRad() - phi)*cos(b.getLatRad())*equatorialRadius);
            v.z = float(height - b.getHeightMSL());

            return v;

        }

        void operator = (Position b) {

            theta = b.theta;
            phi = b.phi;
            height = b.height;

        }


    private:

        const double pi = 3.141592654;

        double getRadius() {

            double st = sin(theta);

            double z = cos(theta) * polarRadius;
            double x = cos(phi)*st * equatorialRadius;
            double y = sin(phi)*st * equatorialRadius;

            double r = sqrt(x*x + y*y + z*z);

            if (r==r) { //Check for NaN
                return r; //return r if no NaN
            } else {
                return 0.0; //return 0 if NaN
            }

        }

};




#endif /* _HELPER_3DMATH_H_ */