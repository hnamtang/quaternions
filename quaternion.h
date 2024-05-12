#include <iostream>
#include <cmath>

class Quaternion
{
protected:
    float m_a, m_b, m_c, m_d;

public:
    Quaternion()  // default constructor
        : m_a(0.0f), m_b(0.0f), m_c(0.0f), m_d(0.0f) {}

    Quaternion(const float& a, const float& b, const float& c, const float& d)
        : m_a(a), m_b(b), m_c(c), m_d(d) {}
    
    ~Quaternion() {}  // destructor

    static const void print(const Quaternion& q);

    void setComponents(const float& a, const float& b, const float& c, const float& d)
    {
        m_a = a;
        m_b = b;
        m_c = c;
        m_d = d;
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

    Quaternion operator*(const Quaternion& other) const
    {
        double a, b, c, d;
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
};

class Vector3 : public Quaternion
{
public:
    Vector3(float x, float y, float z)
        : Quaternion(0.0, x, y, z) {}

    Vector3 operator*(const Vector3& v) const
    {
        float b, c, d;
        b = m_a * v.m_b + m_b * v.m_a
            + m_c * v.m_d - m_d * v.m_c;
        c = m_a * v.m_c - m_b * v.m_d
            + m_c * v.m_a + m_d * v.m_b;
        d = m_a * v.m_d + m_b * v.m_c
            - m_c * v.m_b + m_d * v.m_a;

        return Vector3(b, c, d);
    }
};

const void Quaternion::print(const Quaternion& q)
{
    std::cout << "q = " << q.m_a;
    if (q.m_b < 0)
        std::cout << " - " << std::abs(q.m_b) << 'i';
    else
        std::cout << " + " << std::abs(q.m_b) << 'i';
    if (q.m_c < 0)
        std::cout << " - " << std::abs(q.m_c) << 'j';
    else
        std::cout << " + " << std::abs(q.m_c) << 'j';
    if (q.m_d < 0)
        std::cout << " - " << std::abs(q.m_d) << 'k' << std::endl;
    else
        std::cout << " + " << std::abs(q.m_d) << 'k' << std::endl;
}

Quaternion inverse(Quaternion& q)
{
    const float mag = q.getMag();
    return Quaternion(q.m_a / mag / mag, -q.m_b / mag / mag,
        -q.m_c / mag / mag, -q.m_d / mag / mag);
}

Vector3 rotate(float theta, Vector3& v)
{
    Quaternion q(std::cos(theta * M_PI / 180 / 2), std::sin(theta * M_PI / 180 / 2), std::sin(theta * M_PI / 180 / 2), std::sin(theta * M_PI / 180 / 2));
    Quaternion q_norm = q.normalize();
    Quaternion q_conj = q_norm.conjugate();
    
    Vector3 v_rotated = q_norm * v * q_conj;
    return v_rotated;
}
