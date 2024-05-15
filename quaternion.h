#include <iostream>
#include <string>
#include <tuple>
#include <cmath>

//#define PRINTER(name) Quaternion::print(#name, (name))
#define PRINTER(name) name.print(#name)

class Vector3
{
private:
    float m_x, m_y, m_z;
public:
    Vector3()  // default constructor
        : m_x(0.0f), m_y(0.0f), m_z(0.0f) {}

    Vector3(const float& x, const float& y, const float& z)  // constructor
        : m_x(x), m_y(y), m_z(z) {}

    ~Vector3() {}  // destructor
    
    std::tuple<float, float, float>getComponents() const
    {
        return std::make_tuple(m_x, m_y, m_z);
    }

    float getMag() const
    {
        return std::sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    }

    void normalize()
    {
        const float mag = getMag();
        m_x /= mag;
        m_y /= mag;
        m_z /= mag;
    }

    void print(const std::string& name)
    {
        float eps = 1e-3;
    
        std::cout << name << " = ";
        if (std::abs(m_x) >= eps)
        {
          if (m_x < 0)
              std::cout << " -" << std::abs(m_x) << 'i';
          else
              std::cout << m_x << 'i';
        }
    
        if (std::abs(m_y) >= eps)
        {
          if (m_y < 0)
              std::cout << " - " << std::abs(m_y) << 'j';
          else if (std::abs(m_x) >= eps)
              std::cout << " + " << m_y << 'j';
          else
              std::cout << m_y << 'j';
        }
    
        if (std::abs(m_z) >= eps)
        {
          if (m_z < 0)
              std::cout << " - " << std::abs(m_z) << 'k';
          else if ((std::abs(m_y) >= eps) || (std::abs(m_x) >= eps))
              std::cout << " + " << m_z << 'k';
          else
              std::cout << m_z << 'k';
        }
        std::cout << '\n';
    }
};

class Quaternion
{
private:
    float m_a, m_b, m_c, m_d;

public:
    Quaternion()  // default constructor
        : m_a(0.0f), m_b(0.0f), m_c(0.0f), m_d(0.0f) {}

    Quaternion(const float& a, const float& b, const float& c, const float& d)
        : m_a(a), m_b(b), m_c(c), m_d(d) {}

    Quaternion(Vector3& axis, const float& angle)  // construction with axis-angle
    {
        axis.normalize();
        float axis_i, axis_j, axis_k;
        std::tie(axis_i, axis_j, axis_k) = axis.getComponents();

        float s = std::sin(angle * M_PI / 180 / 2);
        m_a = std::cos(angle * M_PI / 180 / 2);
        m_b = axis_i * s;
        m_c = axis_j * s;
        m_d = axis_k * s;
    }
    
    ~Quaternion() {}  // destructor

//    static void print(const std::string& name, const Quaternion& q);

    void setComponents(const float& a, const float& b, const float& c, const float& d)
    {
        m_a = a;
        m_b = b;
        m_c = c;
        m_d = d;
    }

    std::tuple<float, float, float, float> getComponents() const
    {
        return std::make_tuple(m_a, m_b, m_c, m_d);
    }

    float getMag() const
    {
        return std::sqrt(m_a * m_a + m_b * m_b + m_c * m_c + m_d * m_d);
    }

    void normalize()
    {
        const float mag = getMag();
        m_a /= mag;
        m_b /= mag;
        m_c /= mag;
        m_d /= mag;
    }

    Quaternion conjugate() const
    {
        return Quaternion(m_a, -m_b, -m_c, -m_d);
    }

    Quaternion operator+(const Quaternion& other) const
    {
        return Quaternion(m_a + other.m_a, m_b
            + other.m_b, m_c + other.m_c, m_d + other.m_d);
    }

    Quaternion operator*(const Quaternion& other) const
    {
        float a, b, c, d;
        a = m_a * other.m_a - m_b * other.m_b
            - m_c * other.m_c - m_d * other.m_d;
        b = m_a * other.m_b + m_b * other.m_a
            + m_c * other.m_d - m_d * other.m_c;
        c = m_a * other.m_c - m_b * other.m_d
            + m_c * other.m_a + m_d * other.m_b;
        d = m_a * other.m_d + m_b * other.m_c
            - m_c * other.m_b + m_d * other.m_a;

        return Quaternion(a, b, c, d);
    }

    Quaternion operator*(const Vector3& v) const
    {
        float v_i, v_j, v_k;
        std::tie(v_i, v_j, v_k) = v.getComponents();

        float a, b, c, d;
        a = -m_b * v_i - m_c * v_j - m_d * v_k;
        b = m_a * v_i + m_c * v_k - m_d * v_j;
        c = m_a * v_j - m_b * v_k + m_d * v_i;
        d = m_a * v_k + m_b * v_j - m_c * v_i;

        return Quaternion(a, b, c, d);
    }

    Quaternion operator/(const float& divisor) const
    {
        return Quaternion(m_a / divisor, m_b / divisor, m_c / divisor, m_d / divisor);
    }

    Quaternion inverse() const
    {
        const float mag = getMag();
        Quaternion q_inv = this->conjugate() / (mag * mag);
        return q_inv;
    }

    void print(const std::string& name)
    {
        float eps = 1e-3;
    
        std::cout << name << " = ";
        if (std::abs(m_a) >= eps)
        {
            if (m_a < 0)
                std::cout << " - " << std::abs(m_a);
            else
                std::cout << m_a;
        }
    
        if (std::abs(m_b) >= eps)
        {
          if (m_b < 0)
              std::cout << " - " << std::abs(m_b) << 'i';
          else if (std::abs(m_a) >= eps)
              std::cout << " + " << m_b << 'i';
          else
              std::cout << m_b << 'i';
        }
    
        if (std::abs(m_c) >= eps)
        {
          if (m_c < 0)
              std::cout << " - " << std::abs(m_c) << 'j';
          else if ((std::abs(m_a) >= eps) || (std::abs(m_b) >= eps))
              std::cout << " + " << m_c << 'j';
          else
              std::cout << m_c << 'j';

        }
    
        if (std::abs(m_d) >= eps)
        {
          if (m_d < 0)
              std::cout << " - " << std::abs(m_d) << 'k';
          else if ((std::abs(m_a) >= eps) || (std::abs(m_b) >= eps) || (std::abs(m_c) >= eps))
              std::cout << " + " << m_d << 'k';
          else
              std::cout << m_d << 'k';
        }
        std::cout << '\n';
    }
};

//void Quaternion::print(const std::string& name, const Quaternion& q)
//{
//    float eps = 1e-3;
//
//    std::cout << name << " = ";
//    if (std::abs(q.m_a) >= eps)
//    {
//        if (q.m_a < 0)
//            std::cout << " - " << std::abs(q.m_a);
//        else
//            std::cout << q.m_a;
//    }
//
//    if (std::abs(q.m_b) >= eps)
//    {
//      if (q.m_b < 0)
//          std::cout << " - " << std::abs(q.m_b) << 'i';
//      else if (std::abs(q.m_a) >= eps)
//          std::cout << " + " << q.m_b << 'i';
//      else
//          std::cout << q.m_b << 'i';
//    }
//
//    if (std::abs(q.m_c) >= eps)
//    {
//      if (q.m_c < 0)
//          std::cout << " - " << std::abs(q.m_c) << 'j';
//      else
//          std::cout << " + " << q.m_c << 'j';
//    }
//
//    if (std::abs(q.m_d) >= eps)
//    {
//      if (q.m_d < 0)
//          std::cout << " - " << std::abs(q.m_d) << 'k';
//      else
//          std::cout << " + " << q.m_d << 'k';
//    }
//    std::cout << '\n';
//}


Vector3 rotate(const Vector3& v, Vector3& axis, const float& angle)
{
    Quaternion q(axis, angle);
    q.normalize();
    Quaternion q_conj = q.conjugate();

    Quaternion v_rotated_in_quaternion = q * v * q_conj;
    float a, x, y, z;
    std::tie(a, x, y, z) = v_rotated_in_quaternion.getComponents();

    Vector3 v_rotated(x, y, z);
    return v_rotated;
}
