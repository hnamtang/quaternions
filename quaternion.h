#include <iostream>
#include <string>
#include <tuple>
#include <cmath>

#define PRINTER(name) Quaternion::print(#name, (name))

class Quaternion
{
protected:
    float m_a, m_b, m_c, m_d;

public:
    Quaternion()  // default constructor
        : m_a(0.0f), m_b(0.0f), m_c(0.0f), m_d(0.0f) {}

    Quaternion(const float& a, const float& b, const float& c, const float& d)
        : m_a(a), m_b(b), m_c(c), m_d(d) {}
    
    virtual ~Quaternion() {}  // destructor

    static const void print(const std::string& name, const Quaternion& q);

    void setComponents(const float& a, const float& b, const float& c, const float& d)
    {
        m_a = a;
        m_b = b;
        m_c = c;
        m_d = d;
    }

    const std::tuple<float, float, float, float> getComponents() const
    {
        return std::make_tuple(m_a, m_b, m_c, m_d);
    }

    const float getMag()
    {
        float mag = std::sqrt(m_a * m_a + m_b * m_b + m_c * m_c + m_d * m_d);
        return mag;
    }
    Quaternion conjugate() const
    {
        return Quaternion(m_a, -m_b, -m_c, -m_d);
    }

    Quaternion normalize()
    {
        float mag = getMag();
        return Quaternion(m_a / mag, m_b / mag, m_c / mag, m_d / mag);
    }

    Quaternion operator+(const Quaternion& other) const
    {
        return Quaternion(m_a + other.m_a, m_b
            + other.m_b, m_c + other.m_c, m_d + other.m_d);
    }

    Quaternion operator*(const Quaternion* other) const
    {
        double a, b, c, d;
        a = m_a * other->m_a - m_b * other->m_b
            - m_c * other->m_c - m_d * other->m_d;
        b = m_a * other->m_b + m_b * other->m_a
            + m_c * other->m_d - m_d * other->m_c;
        c = m_a * other->m_c - m_b * other->m_d
            + m_c * other->m_a + m_d * other->m_b;
        d = m_a * other->m_d + m_b * other->m_c
            - m_c * other->m_b + m_d * other->m_a;

        return Quaternion(a, b, c, d);
    }
    Quaternion operator/(const float& divisor) const
    {
        return Quaternion(m_a / divisor, m_b / divisor, m_c / divisor, m_d / divisor);
    }
};

class Vector3 : public Quaternion
{
public:
    Vector3(float x, float y, float z)  // constructor
        : Quaternion(0.0, x, y, z) {}

    ~Vector3() {}  // destructor
    
    const std::tuple<float, float, float>getComponents() const
    {
        return std::make_tuple(m_b, m_c, m_d);
    }
};

const void Quaternion::print(const std::string& name, const Quaternion& q)
{
    float eps = 1e-3;

    std::cout << name << " = ";
    if (std::abs(q.m_a) >= eps)
    {
        if (q.m_a < 0)
            std::cout << " - " << std::abs(q.m_a);
        else
            std::cout << q.m_a;
    }

    if (std::abs(q.m_b) >= eps)
    {
      if (q.m_b < 0)
          std::cout << " - " << std::abs(q.m_b) << 'i';
      else if (std::abs(q.m_a) >= eps)
          std::cout << " + " << q.m_b << 'i';
      else
          std::cout << q.m_b << 'i';
    }

    if (std::abs(q.m_c) >= eps)
    {
      if (q.m_c < 0)
          std::cout << " - " << std::abs(q.m_c) << 'j';
      else
          std::cout << " + " << q.m_c << 'j';
    }

    if (std::abs(q.m_d) >= eps)
    {
      if (q.m_d < 0)
          std::cout << " - " << std::abs(q.m_d) << 'k';
      else
          std::cout << " + " << q.m_d << 'k';
    }
    std::cout << '\n';
}

Quaternion inverse(Quaternion& q)
{
    const float mag = q.getMag();
    return q.conjugate() / (mag * mag);
}

Quaternion rotate(const Vector3& v, const Vector3& axis, const float& theta)
{
    float axis_b, axis_c, axis_d;
    std::tie(axis_b, axis_c, axis_d) = axis.getComponents();
    Quaternion q(std::cos(theta * M_PI / 180 / 2), std::sin(theta * M_PI / 180 / 2) * axis_b,\
        std::sin(theta * M_PI / 180 / 2) * axis_c, std::sin(theta * M_PI / 180 / 2) * axis_d);
    Quaternion q_norm = q.normalize();
    Quaternion q_conj = q_norm.conjugate();

    Quaternion v_rotated_in_quaternion = q_norm * &v * &q_conj;
    return v_rotated_in_quaternion;
}
