// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


#include "helper_3dmath.h"
#include "math.h"

void QuaternionInit(Quaternion* quat)
{
    quat->w = 1.0f;
    quat->x = 0.0f;
    quat->y = 0.0f;
    quat->z = 0.0f;
}

Quaternion QuaternionGetProduct(Quaternion* main, Quaternion* second)
{
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    Quaternion product;
    product.w = main->w * second->w - main->x * second->x - main->y * second->y - main->z * second->z;  // new w
    product.x = main->w * second->x + main->x * second->w + main->y * second->z - main->z * second->y;  // new x
    product.y = main->w * second->y - main->x * second->z + main->y * second->w + main->z * second->x;  // new y
    product.z = main->w * second->z + main->x * second->y - main->y * second->x + main->z * second->w; // new z

    return product;
}


Quaternion QuaternionGetConjugate(Quaternion* quat) 
{
    Quaternion conjugate;
    conjugate.w = quat->w;
    conjugate.x = -quat->x;
    conjugate.y = -quat->y;
    conjugate.z = -quat->z;
    return conjugate;
}
        
float QuaternionGetMagnitude(Quaternion* quat)
{
    return sqrtf(quat->w * quat->w + quat->x * quat->x + quat->y * quat->y + quat->z * quat->z);
}
        
void QuaternionNormalize(Quaternion* quat) 
{
    float m = QuaternionGetMagnitude(quat);
    quat->w /= m;
    quat->x /= m;
    quat->y /= m;
    quat->z /= m;
}

Quaternion QuaternionGetNormalized(Quaternion* quat)
{
    Quaternion normalized;
    normalized.w = quat->w;
    normalized.x = quat->x;
    normalized.y = quat->y;
    normalized.z = quat->z;

    QuaternionNormalize(&normalized);
    
    return normalized;
}


void VectorInt16Init(VectorInt16* vector) 
{
    vector->x = 0.0f;
    vector->y = 0.0f;
    vector->z = 0.0f;
}

float VectorInt16GetMagnitude(VectorInt16* vector) 
{
    return sqrt(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
}

void VectorInt16Normalize(VectorInt16* vector) 
{
    float m = VectorInt16GetMagnitude(vector);
    vector->x /= m;
    vector->y /= m;
    vector->z /= m;
}
        
VectorInt16 VectorInt16GetNormalized(VectorInt16* vector) 
{
    VectorInt16 normalized;
    normalized.x = vector->x;
    normalized.y = vector->y;
    normalized.z = vector->z;

    VectorInt16Normalize(&normalized);
    return normalized;
}
        
void VectorInt16Rotate(VectorInt16* vector, Quaternion *q) 
{
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p;
    p.w = 0;
    p.x = vector->x;
    p.y = vector->y;
    p.z = vector->z;

    // quaternion multiplication: q * p, stored back in p
    p = QuaternionGetProduct(q, &p);

    // quaternion multiplication: p * conj(q), stored back in p
    Quaternion conjugate = QuaternionGetConjugate(q);
    p = QuaternionGetProduct(&p, &conjugate);

    // p quaternion is now [0, x', y', z']
    vector->x = p.x;
    vector->y = p.y;
    vector->z = p.z;
}

VectorInt16 VectorInt16GetRotated(VectorInt16* vector, Quaternion *q) 
{
    VectorInt16 r;
    r.x = vector->x;
    r.y = vector->y;
    r.z = vector->z;

    VectorInt16Rotate(&r, q);
    return r;
}