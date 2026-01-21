#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include <cmath>

struct Vector3
{
    float x;
    float y;
    float z;

    Vector3 operator+(const Vector3& b) const
    {
        return Vector3{x + b.x, y + b.y, z + b.z};
    }

    Vector3 operator-(const Vector3& b) const
    {
        return Vector3{x - b.x, y - b.y, z - b.z};
    }

    Vector3 operator/(const float b) const
    {
        if (b == 0)
        {
            return Vector3{0, 0, 0};
        }
        return Vector3{x/b, y/b, z/b};
    }
};



struct Quaternion
{
    float w, x, y, z;
    
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quaternion conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    void normalize()
    {
        float mag = std::sqrt(w*w + x*x + y*y + z*z);
        if (mag > 0.0f)
        {
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        }
    }
    
    // Rotate a vector using this quaternion: q * v * q^-1
    Vector3 rotate_vector(const Vector3& v) const
    {
        // Convert vector to quaternion
        Quaternion v_quat(0.0f, v.x, v.y, v.z);
        
        // Compute conjugate (inverse for unit quaternions)
        Quaternion q_conj(w, -x, -y, -z);
        
        // q * v * q^-1
        Quaternion result = *this * v_quat;
        result = result * q_conj;
        
        return Vector3{result.x, result.y, result.z};
    }
    
    // Rotate a vector using the inverse of this quaternion: q^-1 * v * q
    Vector3 rotate_vector_inverse(const Vector3& v) const
    {
        // Compute conjugate (inverse for unit quaternions)
        Quaternion q_conj(w, -x, -y, -z);
        return q_conj.rotate_vector(v);
    }
    
    Quaternion operator*(const Quaternion& b) const
    {
        return Quaternion(
            w*b.w - x*b.x - y*b.y - z*b.z,
            w*b.x + x*b.w + y*b.z - z*b.y,
            w*b.y - x*b.z + y*b.w + z*b.x,
            w*b.z + x*b.y - y*b.x + z*b.w
        );
    }
};

#endif // STRUCTS_HPP