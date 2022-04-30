#include <math.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#define M_PI 3.1415926535

#define ToRadian(x) ((x) * M_PI / 180.0f)
#define ToDegree(x) ((x) * 180.0f / M_PI)

struct Vector2i
{
    int x;
    int y;
};

struct Vector2f
{
    float x;
    float y;

    Vector2f()
    {
    }

    Vector2f(float _x, float _y)
    {
        x = _x;
        y = _y;
    }
};


struct Vector3f
{
    float x;
    float y;
    float z;

    Vector3f()
    {
    }

    Vector3f(float _x, float _y, float _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    Vector3f& operator+=(const Vector3f& r)
    {
        x += r.x;
        y += r.y;
        z += r.z;

        return *this;
    }

    Vector3f& operator-=(const Vector3f& r)
    {
        x -= r.x;
        y -= r.y;
        z -= r.z;

        return *this;
    }

    Vector3f& operator*=(float f)
    {
        x *= f;
        y *= f;
        z *= f;

        return *this;
    }

    Vector3f Cross(const Vector3f& v) const;

    Vector3f& Normalize();

    void Rotate(float Angle, const Vector3f& Axis);

    void Print() const
    {
        printf("(%.02f, %.02f, %.02f", x, y, z);
    }
};


inline Vector3f operator+(const Vector3f& l, const Vector3f& r)
{
    Vector3f Ret(l.x + r.x,
        l.y + r.y,
        l.z + r.z);

    return Ret;
}

inline Vector3f operator-(const Vector3f& l, const Vector3f& r)
{
    Vector3f Ret(l.x - r.x,
        l.y - r.y,
        l.z - r.z);

    return Ret;
}

inline Vector3f operator*(const Vector3f& l, float f)
{
    Vector3f Ret(l.x * f,
        l.y * f,
        l.z * f);

    return Ret;
}


class Matrix4f
{
public:
    float m[4][4];

    Matrix4f()
    {
    }


    inline void InitIdentity()
    {
        m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f;
        m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f;
        m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f;
        m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
    }

    inline Matrix4f operator*(const Matrix4f& Right) const
    {
        Matrix4f Ret;

        for (unsigned int i = 0; i < 4; i++) {
            for (unsigned int j = 0; j < 4; j++) {
                Ret.m[i][j] = m[i][0] * Right.m[0][j] +
                    m[i][1] * Right.m[1][j] +
                    m[i][2] * Right.m[2][j] +
                    m[i][3] * Right.m[3][j];
            }
        }

        return Ret;
    }

    void InitScaleTransform(float ScaleX, float ScaleY, float ScaleZ);
    void InitRotateTransform(float RotateX, float RotateY, float RotateZ);
    void InitTranslationTransform(float x, float y, float z);
    void InitCameraTransform(const Vector3f& Target, const Vector3f& Up);
    void InitPersProjTransform(float FOV, float Width, float Height, float zNear, float zFar);
};


struct Quaternion
{
    float x, y, z, w;

    Quaternion(float _x, float _y, float _z, float _w);

    void Normalize();

    Quaternion Conjugate();
};

Quaternion operator*(const Quaternion& l, const Quaternion& r);

Quaternion operator*(const Quaternion& q, const Vector3f& v);
Vector3f Vector3f::Cross(const Vector3f& v) const
{
    const float _x = y * v.z - z * v.y;
    const float _y = z * v.x - x * v.z;
    const float _z = x * v.y - y * v.x;

    return Vector3f(_x, _y, _z);
}

Vector3f& Vector3f::Normalize()
{
    const float Length = sqrtf(x * x + y * y + z * z);

    x /= Length;
    y /= Length;
    z /= Length;

    return *this;
}

void Vector3f::Rotate(float Angle, const Vector3f& Axe)
{
    const float SinHalfAngle = sinf(ToRadian(Angle / 2));
    const float CosHalfAngle = cosf(ToRadian(Angle / 2));

    const float Rx = Axe.x * SinHalfAngle;
    const float Ry = Axe.y * SinHalfAngle;
    const float Rz = Axe.z * SinHalfAngle;
    const float Rw = CosHalfAngle;
    Quaternion RotationQ(Rx, Ry, Rz, Rw);

    Quaternion ConjugateQ = RotationQ.Conjugate();
    //  ConjugateQ.Normalize();
    Quaternion W = RotationQ * (*this) * ConjugateQ;

    x = W.x;
    y = W.y;
    z = W.z;
}


void Matrix4f::InitScaleTransform(float ScaleX, float ScaleY, float ScaleZ)
{
    m[0][0] = ScaleX; m[0][1] = 0.0f;   m[0][2] = 0.0f;   m[0][3] = 0.0f;
    m[1][0] = 0.0f;   m[1][1] = ScaleY; m[1][2] = 0.0f;   m[1][3] = 0.0f;
    m[2][0] = 0.0f;   m[2][1] = 0.0f;   m[2][2] = ScaleZ; m[2][3] = 0.0f;
    m[3][0] = 0.0f;   m[3][1] = 0.0f;   m[3][2] = 0.0f;   m[3][3] = 1.0f;
}

void Matrix4f::InitRotateTransform(float RotateX, float RotateY, float RotateZ)
{
    Matrix4f rx, ry, rz;

    const float x = ToRadian(RotateX);
    const float y = ToRadian(RotateY);
    const float z = ToRadian(RotateZ);

    rx.m[0][0] = 1.0f; rx.m[0][1] = 0.0f; rx.m[0][2] = 0.0f; rx.m[0][3] = 0.0f;
    rx.m[1][0] = 0.0f; rx.m[1][1] = cosf(x); rx.m[1][2] = -sinf(x); rx.m[1][3] = 0.0f;
    rx.m[2][0] = 0.0f; rx.m[2][1] = sinf(x); rx.m[2][2] = cosf(x); rx.m[2][3] = 0.0f;
    rx.m[3][0] = 0.0f; rx.m[3][1] = 0.0f; rx.m[3][2] = 0.0f; rx.m[3][3] = 1.0f;

    ry.m[0][0] = cosf(y); ry.m[0][1] = 0.0f; ry.m[0][2] = -sinf(y); ry.m[0][3] = 0.0f;
    ry.m[1][0] = 0.0f; ry.m[1][1] = 1.0f; ry.m[1][2] = 0.0f; ry.m[1][3] = 0.0f;
    ry.m[2][0] = sinf(y); ry.m[2][1] = 0.0f; ry.m[2][2] = cosf(y); ry.m[2][3] = 0.0f;
    ry.m[3][0] = 0.0f; ry.m[3][1] = 0.0f; ry.m[3][2] = 0.0f; ry.m[3][3] = 1.0f;

    rz.m[0][0] = cosf(z); rz.m[0][1] = -sinf(z); rz.m[0][2] = 0.0f; rz.m[0][3] = 0.0f;
    rz.m[1][0] = sinf(z); rz.m[1][1] = cosf(z); rz.m[1][2] = 0.0f; rz.m[1][3] = 0.0f;
    rz.m[2][0] = 0.0f; rz.m[2][1] = 0.0f; rz.m[2][2] = 1.0f; rz.m[2][3] = 0.0f;
    rz.m[3][0] = 0.0f; rz.m[3][1] = 0.0f; rz.m[3][2] = 0.0f; rz.m[3][3] = 1.0f;

    *this = rz * ry * rx;
}

void Matrix4f::InitTranslationTransform(float x, float y, float z)
{
    m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = x;
    m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = y;
    m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = z;
    m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f;
}


void Matrix4f::InitCameraTransform(const Vector3f& Target, const Vector3f& Up)
{
    Vector3f N = Target;
    N.Normalize();
    Vector3f U = Up;
    U.Normalize();
    U = U.Cross(N);
    Vector3f V = N.Cross(U);

    m[0][0] = U.x;   m[0][1] = U.y;   m[0][2] = U.z;   m[0][3] = 0.0f;
    m[1][0] = V.x;   m[1][1] = V.y;   m[1][2] = V.z;   m[1][3] = 0.0f;
    m[2][0] = N.x;   m[2][1] = N.y;   m[2][2] = N.z;   m[2][3] = 0.0f;
    m[3][0] = 0.0f;  m[3][1] = 0.0f;  m[3][2] = 0.0f;  m[3][3] = 1.0f;
}

void Matrix4f::InitPersProjTransform(float FOV, float Width, float Height, float zNear, float zFar)
{
    const float ar = Width / Height;
    const float zRange = zNear - zFar;
    const float tanHalfFOV = tanf(ToRadian(FOV / 2.0f));

    m[0][0] = 1.0f / (tanHalfFOV * ar); m[0][1] = 0.0f;            m[0][2] = 0.0f;          m[0][3] = 0.0;
    m[1][0] = 0.0f;                   m[1][1] = 1.0f / tanHalfFOV; m[1][2] = 0.0f;          m[1][3] = 0.0;
    m[2][0] = 0.0f;                   m[2][1] = 0.0f;            m[2][2] = (-zNear - zFar) / zRange; m[2][3] = 2.0f * zFar * zNear / zRange;
    m[3][0] = 0.0f;                   m[3][1] = 0.0f;            m[3][2] = 1.0f;          m[3][3] = 0.0;
}


Quaternion::Quaternion(float _x, float _y, float _z, float _w)
{
    x = _x;
    y = _y;
    z = _z;
    w = _w;
}

void Quaternion::Normalize()
{
    float Length = sqrtf(x * x + y * y + z * z + w * w);

    x /= Length;
    y /= Length;
    z /= Length;
    w /= Length;
}


Quaternion Quaternion::Conjugate()
{
    Quaternion ret(-x, -y, -z, w);
    return ret;
}

Quaternion operator*(const Quaternion& l, const Quaternion& r)
{
    const float w = (l.w * r.w) - (l.x * r.x) - (l.y * r.y) - (l.z * r.z);
    const float x = (l.x * r.w) + (l.w * r.x) + (l.y * r.z) - (l.z * r.y);
    const float y = (l.y * r.w) + (l.w * r.y) + (l.z * r.x) - (l.x * r.z);
    const float z = (l.z * r.w) + (l.w * r.z) + (l.x * r.y) - (l.y * r.x);

    Quaternion ret(x, y, z, w);

    return ret;
}

Quaternion operator*(const Quaternion& q, const Vector3f& v)
{
    const float w = -(q.x * v.x) - (q.y * v.y) - (q.z * v.z);
    const float x = (q.w * v.x) + (q.y * v.z) - (q.z * v.y);
    const float y = (q.w * v.y) + (q.z * v.x) - (q.x * v.z);
    const float z = (q.w * v.z) + (q.x * v.y) - (q.y * v.x);

    Quaternion ret(x, y, z, w);

    return ret;
}
class Pipeline
{
public:
    Pipeline()
    {
        m_scale = Vector3f(1.0f, 1.0f, 1.0f);
        m_worldPos = Vector3f(0.0f, 0.0f, 0.0f);
        m_rotateInfo = Vector3f(0.0f, 0.0f, 0.0f);
    }

    void Scale(float ScaleX, float ScaleY, float ScaleZ)
    {
        m_scale.x = ScaleX;
        m_scale.y = ScaleY;
        m_scale.z = ScaleZ;
    }

    void WorldPos(float x, float y, float z)
    {
        m_worldPos.x = x;
        m_worldPos.y = y;
        m_worldPos.z = z;
    }

    void Rotate(float RotateX, float RotateY, float RotateZ)
    {
        m_rotateInfo.x = RotateX;
        m_rotateInfo.y = RotateY;
        m_rotateInfo.z = RotateZ;
    }

    void SetPerspectiveProj(float FOV, float Width, float Height, float zNear, float zFar)
    {
        m_persProj.FOV = FOV;
        m_persProj.Width = Width;
        m_persProj.Height = Height;
        m_persProj.zNear = zNear;
        m_persProj.zFar = zFar;
    }

    void SetCamera(const Vector3f& Pos, const Vector3f& Target, const Vector3f& Up)
    {
        m_camera.Pos = Pos;
        m_camera.Target = Target;
        m_camera.Up = Up;
    }


    const Matrix4f& GetWVPTrans();

    const Matrix4f& GetWorldTrans();


private:
    Vector3f m_scale;
    Vector3f m_worldPos;
    Vector3f m_rotateInfo;

    struct {
        float FOV;
        float Width;
        float Height;
        float zNear;
        float zFar;
    } m_persProj;

    struct {
        Vector3f Pos;
        Vector3f Target;
        Vector3f Up;
    } m_camera;

    Matrix4f m_WVPtransformation;
    Matrix4f m_WorldTransformation;
};
const Matrix4f& Pipeline::GetWorldTrans()
{
    Matrix4f ScaleTrans, RotateTrans, TranslationTrans;

    ScaleTrans.InitScaleTransform(m_scale.x, m_scale.y, m_scale.z);
    RotateTrans.InitRotateTransform(m_rotateInfo.x, m_rotateInfo.y, m_rotateInfo.z);
    TranslationTrans.InitTranslationTransform(m_worldPos.x, m_worldPos.y, m_worldPos.z);

    m_WorldTransformation = TranslationTrans * RotateTrans * ScaleTrans;
    return m_WorldTransformation;
}

const Matrix4f& Pipeline::GetWVPTrans()
{
    GetWorldTrans();

    Matrix4f CameraTranslationTrans, CameraRotateTrans, PersProjTrans;

    CameraTranslationTrans.InitTranslationTransform(-m_camera.Pos.x, -m_camera.Pos.y, -m_camera.Pos.z);
    CameraRotateTrans.InitCameraTransform(m_camera.Target, m_camera.Up);
    PersProjTrans.InitPersProjTransform(m_persProj.FOV, m_persProj.Width, m_persProj.Height, m_persProj.zNear, m_persProj.zFar);

    m_WVPtransformation = PersProjTrans * CameraRotateTrans * CameraTranslationTrans * m_WorldTransformation;
    return m_WVPtransformation;
}
class Camera
{
public:

    Camera(int WindowWidth, int WindowHeight);

    Camera(int WindowWidth, int WindowHeight, const Vector3f& Pos, const Vector3f& Target, const Vector3f& Up);

    bool OnKeyboard(int Key);

    void OnMouse(int x, int y);

    void OnRender();

    const Vector3f& GetPos() const
    {
        return m_pos;
    }

    const Vector3f& GetTarget() const
    {
        return m_target;
    }

    const Vector3f& GetUp() const
    {
        return m_up;
    }

private:

    void Init();
    void Update();

    Vector3f m_pos;
    Vector3f m_target;
    Vector3f m_up;

    int m_windowWidth;
    int m_windowHeight;

    float m_AngleH;
    float m_AngleV;

    Vector2i m_mousePos;
};
const static float STEP_SCALE = 0.1f;
const static int MARGIN = 10;

Camera::Camera(int WindowWidth, int WindowHeight)
{
    m_windowWidth = WindowWidth;
    m_windowHeight = WindowHeight;
    m_pos = Vector3f(0.0f, 0.0f, 0.0f);
    m_target = Vector3f(0.0f, 0.0f, 1.0f);
    m_target.Normalize();
    m_up = Vector3f(0.0f, 1.0f, 0.0f);

    Init();
}


Camera::Camera(int WindowWidth, int WindowHeight, const Vector3f& Pos, const Vector3f& Target, const Vector3f& Up)
{
    m_windowWidth = WindowWidth;
    m_windowHeight = WindowHeight;
    m_pos = Pos;

    m_target = Target;
    m_target.Normalize();

    m_up = Up;
    m_up.Normalize();

    Init();
}


void Camera::Init()
{
    Vector3f HTarget(m_target.x, 0.0, m_target.z);
    HTarget.Normalize();

    if (HTarget.z >= 0.0f)
    {
        if (HTarget.x >= 0.0f)
        {
            m_AngleH = 360.0f - ToDegree(asin(HTarget.z));
        }
        else
        {
            m_AngleH = 180.0f + ToDegree(asin(HTarget.z));
        }
    }
    else
    {
        if (HTarget.x >= 0.0f)
        {
            m_AngleH = ToDegree(asin(-HTarget.z));
        }
        else
        {
            m_AngleH = 90.0f + ToDegree(asin(-HTarget.z));
        }
    }

    m_AngleV = -ToDegree(asin(m_target.y));

    m_mousePos.x = m_windowWidth / 2;
    m_mousePos.y = m_windowHeight / 2;

    glutWarpPointer(m_mousePos.x, m_mousePos.y);
}


bool Camera::OnKeyboard(int Key)
{
    bool Ret = false;

    switch (Key) {

    case GLUT_KEY_UP:
    {
        m_pos += (m_target * STEP_SCALE);
        Ret = true;
    }
    break;

    case GLUT_KEY_DOWN:
    {
        m_pos -= (m_target * STEP_SCALE);
        Ret = true;
    }
    break;

    case GLUT_KEY_LEFT:
    {
        Vector3f Left = m_target.Cross(m_up);
        Left.Normalize();
        Left *= STEP_SCALE;
        m_pos += Left;
        Ret = true;
    }
    break;

    case GLUT_KEY_RIGHT:
    {
        Vector3f Right = m_up.Cross(m_target);
        Right.Normalize();
        Right *= STEP_SCALE;
        m_pos += Right;
        Ret = true;
    }
    break;
    }

    return Ret;
}


void Camera::OnMouse(int x, int y)
{
    if ((x == m_mousePos.x) && (y == m_mousePos.y)) return;

    const int DeltaX = x - m_mousePos.x;
    const int DeltaY = y - m_mousePos.y;

    m_AngleH += (float)DeltaX / 20.0f;
    m_AngleV += (float)DeltaY / 20.0f;

    Update();
    glutWarpPointer(m_mousePos.x, m_mousePos.y);
}


void Camera::OnRender()
{
    bool ShouldUpdate = false;

    if (ShouldUpdate) {
        Update();
    }
}

void Camera::Update()
{
    const Vector3f Vaxis(0.0f, 1.0f, 0.0f);

    // Rotate the view vector by the horizontal angle around the vertical axis
    Vector3f View(1.0f, 0.0f, 0.0f);
    View.Rotate(m_AngleH, Vaxis);
    View.Normalize();

    // Rotate the view vector by the vertical angle around the horizontal axis
    Vector3f Haxis = Vaxis.Cross(View);
    Haxis.Normalize();
    View.Rotate(m_AngleV, Haxis);

    m_target = View;
    m_target.Normalize();

    m_up = m_target.Cross(Haxis);
    m_up.Normalize();
}
#define ARRAY_SIZE_IN_ELEMENTS(a) (sizeof(a)/sizeof(a[0]))
#include <string>
#include <Magick++.h>

class Texture
{
public:
    Texture(GLenum TextureTarget, const std::string& FileName);

    bool Load();

    void Bind(GLenum TextureUnit);

private:
    std::string m_fileName;
    GLenum m_textureTarget;
    GLuint m_textureObj;
    Magick::Image* m_pImage;
    Magick::Blob m_blob;
};
Texture::Texture(GLenum TextureTarget, const std::string& FileName)
{
    m_textureTarget = TextureTarget;
    m_fileName = FileName;
    m_pImage = NULL;
}

bool Texture::Load()
{
    try {
        m_pImage = new Magick::Image(m_fileName);
        m_pImage->write(&m_blob, "RGBA");
    }
    catch (Magick::Error& Error) {
        std::cout << "Error loading texture '" << m_fileName << "': " << Error.what() << std::endl;
        return false;
    }

    glGenTextures(1, &m_textureObj);
    glBindTexture(m_textureTarget, m_textureObj);
    glTexImage2D(m_textureTarget, 0, GL_RGB, m_pImage->columns(), m_pImage->rows(), -0.5, GL_RGBA, GL_UNSIGNED_BYTE, m_blob.data());
    glTexParameterf(m_textureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(m_textureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    return true;
}

void Texture::Bind(GLenum TextureUnit)
{
    glActiveTexture(TextureUnit);
    glBindTexture(m_textureTarget, m_textureObj);
}
#include <list>

#define INVALID_UNIFORM_LOCATION 0xFFFFFFFF
#include <stdio.h>
#include <string.h>
class Technique
{
public:
    Technique();
    ~Technique();
    virtual bool Init();
    void Enable();

protected:
    bool AddShader(GLenum ShaderType, const char* pShaderText);
    bool Finalize();
    GLint GetUniformLocation(const char* pUniformName);

private:
    GLuint m_shaderProg;
    typedef std::list<GLuint> ShaderObjList;
    ShaderObjList m_shaderObjList;
};
Technique::Technique() {
    m_shaderProg = 0;
}

Technique::~Technique() {
    for (ShaderObjList::iterator it = m_shaderObjList.begin(); it != m_shaderObjList.end(); it++) {
        glDeleteShader(*it);
    }

    if (m_shaderProg != 0) {
        glDeleteProgram(m_shaderProg);
        m_shaderProg = 0;
    }
}

bool Technique::Init() {
    m_shaderProg = glCreateProgram();

    if (m_shaderProg == 0) {
        fprintf(stderr, "Error creating shader program\n");
        return false;
    }

    return true;
}

//Используем этот метод для добавления шейдеров в программу. Когда заканчиваем - вызываем finalize()
bool Technique::AddShader(GLenum ShaderType, const char* pShaderText) {
    GLuint ShaderObj = glCreateShader(ShaderType);

    if (ShaderObj == 0) {
        fprintf(stderr, "Error creating shader type %d\n", ShaderType);
        return false;
    }

    // Сохраним объект шейдера - он будет удален в декструкторе
    m_shaderObjList.push_back(ShaderObj);

    const GLchar* p[1];
    p[0] = pShaderText;
    GLint Lengths[1];
    Lengths[0] = strlen(pShaderText);
    glShaderSource(ShaderObj, 1, p, Lengths);

    glCompileShader(ShaderObj);

    GLint success;
    glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);

    if (!success) {
        GLchar InfoLog[1024];
        glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
        fprintf(stderr, "Error compiling shader type %d: '%s'\n", ShaderType, InfoLog);
        return false;
    }

    glAttachShader(m_shaderProg, ShaderObj);

    return true;
}

// После добавления всех шейдеров в программу вызываем эту функцию
// для линковки и проверки программу на ошибки
bool Technique::Finalize() {
    GLint Success = 0;
    GLchar ErrorLog[1024] = { 0 };

    glLinkProgram(m_shaderProg);

    glGetProgramiv(m_shaderProg, GL_LINK_STATUS, &Success);
    if (Success == 0) {
        glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
        return false;
    }

    glValidateProgram(m_shaderProg);
    glGetProgramiv(m_shaderProg, GL_VALIDATE_STATUS, &Success);
    if (Success == 0) {
        glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
        fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
        return false;
    }

    // Удаляем промежуточные объекты шейдеров, которые были добавлены в программу
    for (ShaderObjList::iterator it = m_shaderObjList.begin(); it != m_shaderObjList.end(); it++) {
        glDeleteShader(*it);
    }

    m_shaderObjList.clear();

    return true;
}

void Technique::Enable() {
    glUseProgram(m_shaderProg);
}

GLint Technique::GetUniformLocation(const char* pUniformName) {
    GLint Location = glGetUniformLocation(m_shaderProg, pUniformName);

    if (Location == 0xFFFFFFFF) {
        fprintf(stderr, "Warning! Unable to get the location of uniform '%s'\n", pUniformName);
    }

    return Location;
}
struct BaseLight
{
    Vector3f Color;
    float AmbientIntensity;
    float DiffuseIntensity;

    BaseLight()
    {
        Color = Vector3f(0.0f, 0.0f, 0.0f);
        AmbientIntensity = 0.0f;
        DiffuseIntensity = 0.0f;
    }
};

struct DirectionalLight : public BaseLight
{
    Vector3f Direction;

    DirectionalLight()
    {
        Direction = Vector3f(0.0f, 0.0f, 0.0f);
    }
};

struct PointLight : public BaseLight
{
    Vector3f Position;

    struct
    {
        float Constant;
        float Linear;
        float Exp;
    } Attenuation;

    PointLight()
    {
        Position = Vector3f(0.0f, 0.0f, 0.0f);
        Attenuation.Constant = 1.0f;
        Attenuation.Linear = 0.0f;
        Attenuation.Exp = 0.0f;
    }
};

struct SpotLight : public PointLight
{
    Vector3f Direction;
    float Cutoff;

    SpotLight()
    {
        Direction = Vector3f(0.0f, 0.0f, 0.0f);
        Cutoff = 0.0f;
    }
};

class LightingTechnique : public Technique {
public:

    static const unsigned int MAX_POINT_LIGHTS = 2;
    static const unsigned int MAX_SPOT_LIGHTS = 2;

    LightingTechnique();

    virtual bool Init();

    void SetWVP(const Matrix4f& WVP);
    void SetWorldMatrix(const Matrix4f& WVP);
    void SetTextureUnit(unsigned int TextureUnit);
    void SetDirectionalLight(const DirectionalLight& Light);
    void SetPointLights(unsigned int NumLights, const PointLight* pLights);
    void SetSpotLights(unsigned int NumLights, const SpotLight* pLights);
    void SetEyeWorldPos(const Vector3f& EyeWorldPos);
    void SetMatSpecularIntensity(float Intensity);
    void SetMatSpecularPower(float Power);

private:

    GLuint m_WVPLocation;
    GLuint m_WorldMatrixLocation;
    GLuint m_samplerLocation;
    GLuint m_eyeWorldPosLocation;
    GLuint m_matSpecularIntensityLocation;
    GLuint m_matSpecularPowerLocation;
    GLuint m_numPointLightsLocation;
    GLuint m_numSpotLightsLocation;

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Direction;
    } m_dirLightLocation;

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Position;
        struct {
            GLuint Constant;
            GLuint Linear;
            GLuint Exp;
        } Atten;
    } m_pointLightsLocation[MAX_POINT_LIGHTS];

    struct {
        GLuint Color;
        GLuint AmbientIntensity;
        GLuint DiffuseIntensity;
        GLuint Position;
        GLuint Direction;
        GLuint Cutoff;
        struct {
            GLuint Constant;
            GLuint Linear;
            GLuint Exp;
        } Atten;
    } m_spotLightsLocation[MAX_SPOT_LIGHTS];
};



static const char* pVS = "                                                          \n\
#version 330                                                                        \n\
                                                                                    \n\
layout (location = 0) in vec3 Position;                                             \n\
layout (location = 1) in vec2 TexCoord;                                             \n\
layout (location = 2) in vec3 Normal;                                               \n\
                                                                                    \n\
uniform mat4 gWVP;                                                                  \n\
uniform mat4 gWorld;                                                                \n\
                                                                                    \n\
out vec2 TexCoord0;                                                                 \n\
out vec3 Normal0;                                                                   \n\
out vec3 WorldPos0;                                                                 \n\
                                                                                    \n\
void main()                                                                         \n\
{                                                                                   \n\
    gl_Position = gWVP * vec4(Position, 1.0);                                       \n\
    TexCoord0   = TexCoord;                                                         \n\
    Normal0     = (gWorld * vec4(Normal, 0.0)).xyz;                                 \n\
    WorldPos0   = (gWorld * vec4(Position, 1.0)).xyz;                               \n\
}";

static const char* pFS = "                                                          \n\
#version 330                                                                        \n\
                                                                                    \n\
const int MAX_POINT_LIGHTS = 2;                                                     \n\
const int MAX_SPOT_LIGHTS = 2;                                                      \n\
                                                                                    \n\
in vec2 TexCoord0;                                                                  \n\
in vec3 Normal0;                                                                    \n\
in vec3 WorldPos0;                                                                  \n\
                                                                                    \n\
out vec4 FragColor;                                                                 \n\
                                                                                    \n\
struct BaseLight                                                                    \n\
{                                                                                   \n\
    vec3 Color;                                                                     \n\
    float AmbientIntensity;                                                         \n\
    float DiffuseIntensity;                                                         \n\
};                                                                                  \n\
                                                                                    \n\
struct DirectionalLight                                                             \n\
{                                                                                   \n\
    BaseLight Base;                                                          \n\
    vec3 Direction;                                                                 \n\
};                                                                                  \n\
                                                                                    \n\
struct Attenuation                                                                  \n\
{                                                                                   \n\
    float Constant;                                                                 \n\
    float Linear;                                                                   \n\
    float Exp;                                                                      \n\
};                                                                                  \n\
                                                                                    \n\
struct PointLight                                                                           \n\
{                                                                                           \n\
    BaseLight Base;                                                                  \n\
    vec3 Position;                                                                          \n\
    Attenuation Atten;                                                                      \n\
};                                                                                          \n\
                                                                                            \n\
struct SpotLight                                                                            \n\
{                                                                                           \n\
    PointLight Base;                                                                 \n\
    vec3 Direction;                                                                         \n\
    float Cutoff;                                                                           \n\
};                                                                                          \n\
                                                                                            \n\
uniform int gNumPointLights;                                                                \n\
uniform int gNumSpotLights;                                                                 \n\
uniform DirectionalLight gDirectionalLight;                                                 \n\
uniform PointLight gPointLights[MAX_POINT_LIGHTS];                                          \n\
uniform SpotLight gSpotLights[MAX_SPOT_LIGHTS];                                             \n\
uniform sampler2D gSampler;                                                                 \n\
uniform vec3 gEyeWorldPos;                                                                  \n\
uniform float gMatSpecularIntensity;                                                        \n\
uniform float gSpecularPower;                                                               \n\
                                                                                            \n\
vec4 CalcLightInternal( BaseLight Light, vec3 LightDirection, vec3 Normal)            \n\
{                                                                                           \n\
    vec4 AmbientColor = vec4(Light.Color, 1.0f) * Light.AmbientIntensity;                   \n\
    float DiffuseFactor = dot(Normal, -LightDirection);                                     \n\
                                                                                            \n\
    vec4 DiffuseColor  = vec4(0, 0, 0, 0);                                                  \n\
    vec4 SpecularColor = vec4(0, 0, 0, 0);                                                  \n\
                                                                                            \n\
    if (DiffuseFactor > 0) {                                                                \n\
        DiffuseColor = vec4(Light.Color, 1.0f) * Light.DiffuseIntensity * DiffuseFactor;    \n\
                                                                                            \n\
        vec3 VertexToEye = normalize(gEyeWorldPos - WorldPos0);                             \n\
        vec3 LightReflect = normalize(reflect(LightDirection, Normal));                     \n\
        float SpecularFactor = dot(VertexToEye, LightReflect);                              \n\
        SpecularFactor = pow(SpecularFactor, gSpecularPower);                               \n\
        if (SpecularFactor > 0) {                                                           \n\
            SpecularColor = vec4(Light.Color, 1.0f) *                                       \n\
                            gMatSpecularIntensity * SpecularFactor;                         \n\
        }                                                                                   \n\
    }                                                                                       \n\
                                                                                            \n\
    return (AmbientColor + DiffuseColor + SpecularColor);                                   \n\
}                                                                                           \n\
                                                                                            \n\
vec4 CalcDirectionalLight(vec3 Normal)                                                      \n\
{                                                                                           \n\
    return CalcLightInternal(gDirectionalLight.Base, gDirectionalLight.Direction, Normal);  \n\
}                                                                                           \n\
                                                                                            \n\
vec4 CalcPointLight( PointLight l, vec3 Normal)                                       \n\
{                                                                                           \n\
    vec3 LightDirection = WorldPos0 - l.Position;                                           \n\
    float Distance = length(LightDirection);                                                \n\
    LightDirection = normalize(LightDirection);                                             \n\
                                                                                            \n\
    vec4 Color = CalcLightInternal(l.Base, LightDirection, Normal);                         \n\
    float Attenuation =  l.Atten.Constant +                                                 \n\
                         l.Atten.Linear * Distance +                                        \n\
                         l.Atten.Exp * Distance * Distance;                                 \n\
                                                                                            \n\
    return Color / Attenuation;                                                             \n\
}                                                                                           \n\
                                                                                            \n\
vec4 CalcSpotLight( SpotLight l, vec3 Normal)                                         \n\
{                                                                                           \n\
    vec3 LightToPixel = normalize(WorldPos0 - l.Base.Position);                             \n\
    float SpotFactor = dot(LightToPixel, l.Direction);                                      \n\
                                                                                            \n\
    if (SpotFactor > l.Cutoff) {                                                            \n\
        vec4 Color = CalcPointLight(l.Base, Normal);                                        \n\
        return Color * (1.0 - (1.0 - SpotFactor) * 1.0/(1.0 - l.Cutoff));                   \n\
    }                                                                                       \n\
    else {                                                                                  \n\
        return vec4(0,0,0,0);                                                               \n\
    }                                                                                       \n\
}                                                                                           \n\
                                                                                            \n\
void main()                                                                                 \n\
{                                                                                           \n\
    vec3 Normal = normalize(Normal0);                                                       \n\
    vec4 TotalLight = CalcDirectionalLight(Normal);                                         \n\
                                                                                            \n\
    for (int i = 0 ; i < gNumPointLights ; i++) {                                           \n\
        TotalLight += CalcPointLight(gPointLights[i], Normal);                              \n\
    }                                                                                       \n\
                                                                                            \n\
    for (int i = 0 ; i < gNumSpotLights ; i++) {                                            \n\
        TotalLight += CalcSpotLight(gSpotLights[i], Normal);                                \n\
    }                                                                                       \n\
                                                                                            \n\
    FragColor = texture2D(gSampler, TexCoord0.xy) * TotalLight;                             \n\
}";



LightingTechnique::LightingTechnique()
{
}

bool LightingTechnique::Init()
{
    if (!Technique::Init()) {
        return false;
    }

    if (!AddShader(GL_VERTEX_SHADER, pVS)) {
        return false;
    }

    if (!AddShader(GL_FRAGMENT_SHADER, pFS)) {
        return false;
    }

    if (!Finalize()) {
        return false;
    }

    m_WVPLocation = GetUniformLocation("gWVP");
    m_WorldMatrixLocation = GetUniformLocation("gWorld");
    m_samplerLocation = GetUniformLocation("gSampler");
    m_eyeWorldPosLocation = GetUniformLocation("gEyeWorldPos");
    m_dirLightLocation.Color = GetUniformLocation("gDirectionalLight.Base.Color");
    m_dirLightLocation.AmbientIntensity = GetUniformLocation("gDirectionalLight.Base.AmbientIntensity");
    m_dirLightLocation.Direction = GetUniformLocation("gDirectionalLight.Direction");
    m_dirLightLocation.DiffuseIntensity = GetUniformLocation("gDirectionalLight.Base.DiffuseIntensity");
    m_matSpecularIntensityLocation = GetUniformLocation("gMatSpecularIntensity");
    m_matSpecularPowerLocation = GetUniformLocation("gSpecularPower");
    m_numPointLightsLocation = GetUniformLocation("gNumPointLights");
    m_numSpotLightsLocation = GetUniformLocation("gNumSpotLights");

    if (m_dirLightLocation.AmbientIntensity == INVALID_UNIFORM_LOCATION ||
        m_WVPLocation == INVALID_UNIFORM_LOCATION ||
        m_WorldMatrixLocation == INVALID_UNIFORM_LOCATION ||
        m_samplerLocation == INVALID_UNIFORM_LOCATION ||
        m_eyeWorldPosLocation == INVALID_UNIFORM_LOCATION ||
        m_dirLightLocation.Color == INVALID_UNIFORM_LOCATION ||
        m_dirLightLocation.DiffuseIntensity == INVALID_UNIFORM_LOCATION ||
        m_dirLightLocation.Direction == INVALID_UNIFORM_LOCATION ||
        m_matSpecularIntensityLocation == INVALID_UNIFORM_LOCATION ||
        m_matSpecularPowerLocation == INVALID_UNIFORM_LOCATION ||
        m_numPointLightsLocation == INVALID_UNIFORM_LOCATION ||
        m_numSpotLightsLocation == INVALID_UNIFORM_LOCATION) {
        return false;
    }

    for (unsigned int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_pointLightsLocation); i++) {
        char Name[128];
        memset(Name, 0, sizeof(Name));
        snprintf(Name, sizeof(Name), "gPointLights[%d].Base.Color", i);
        m_pointLightsLocation[i].Color = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Base.AmbientIntensity", i);
        m_pointLightsLocation[i].AmbientIntensity = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Position", i);
        m_pointLightsLocation[i].Position = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Base.DiffuseIntensity", i);
        m_pointLightsLocation[i].DiffuseIntensity = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Atten.Constant", i);
        m_pointLightsLocation[i].Atten.Constant = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Atten.Linear", i);
        m_pointLightsLocation[i].Atten.Linear = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gPointLights[%d].Atten.Exp", i);
        m_pointLightsLocation[i].Atten.Exp = GetUniformLocation(Name);

        if (m_pointLightsLocation[i].Color == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].AmbientIntensity == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].Position == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].DiffuseIntensity == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].Atten.Constant == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].Atten.Linear == INVALID_UNIFORM_LOCATION ||
            m_pointLightsLocation[i].Atten.Exp == INVALID_UNIFORM_LOCATION) {
            return false;
        }
    }

    for (unsigned int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(m_spotLightsLocation); i++) {
        char Name[128];
        memset(Name, 0, sizeof(Name));
        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Base.Color", i);
        m_spotLightsLocation[i].Color = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Base.AmbientIntensity", i);
        m_spotLightsLocation[i].AmbientIntensity = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Position", i);
        m_spotLightsLocation[i].Position = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Direction", i);
        m_spotLightsLocation[i].Direction = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Cutoff", i);
        m_spotLightsLocation[i].Cutoff = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Base.DiffuseIntensity", i);
        m_spotLightsLocation[i].DiffuseIntensity = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Atten.Constant", i);
        m_spotLightsLocation[i].Atten.Constant = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Atten.Linear", i);
        m_spotLightsLocation[i].Atten.Linear = GetUniformLocation(Name);

        snprintf(Name, sizeof(Name), "gSpotLights[%d].Base.Atten.Exp", i);
        m_spotLightsLocation[i].Atten.Exp = GetUniformLocation(Name);

        if (m_spotLightsLocation[i].Color == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].AmbientIntensity == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Position == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Direction == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Cutoff == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].DiffuseIntensity == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Atten.Constant == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Atten.Linear == INVALID_UNIFORM_LOCATION ||
            m_spotLightsLocation[i].Atten.Exp == INVALID_UNIFORM_LOCATION) {
            return false;
        }
    }

    return true;
}

void LightingTechnique::SetWVP(const Matrix4f& WVP)
{
    glUniformMatrix4fv(m_WVPLocation, 1, GL_TRUE, (const GLfloat*)WVP.m);
}


void LightingTechnique::SetWorldMatrix(const Matrix4f& WorldInverse)
{
    glUniformMatrix4fv(m_WorldMatrixLocation, 1, GL_TRUE, (const GLfloat*)WorldInverse.m);
}


void LightingTechnique::SetTextureUnit(unsigned int TextureUnit)
{
    glUniform1i(m_samplerLocation, TextureUnit);
}


void LightingTechnique::SetDirectionalLight(const DirectionalLight& Light)
{
    glUniform3f(m_dirLightLocation.Color, Light.Color.x, Light.Color.y, Light.Color.z);
    glUniform1f(m_dirLightLocation.AmbientIntensity, Light.AmbientIntensity);
    Vector3f Direction = Light.Direction;
    Direction.Normalize();
    glUniform3f(m_dirLightLocation.Direction, Direction.x, Direction.y, Direction.z);
    glUniform1f(m_dirLightLocation.DiffuseIntensity, Light.DiffuseIntensity);
}

void LightingTechnique::SetEyeWorldPos(const Vector3f& EyeWorldPos)
{
    glUniform3f(m_eyeWorldPosLocation, EyeWorldPos.x, EyeWorldPos.y, EyeWorldPos.z);
}

void LightingTechnique::SetMatSpecularIntensity(float Intensity)
{
    glUniform1f(m_matSpecularIntensityLocation, Intensity);
}

void LightingTechnique::SetMatSpecularPower(float Power)
{
    glUniform1f(m_matSpecularPowerLocation, Power);
}

void LightingTechnique::SetPointLights(unsigned int NumLights, const PointLight* pLights)
{
    glUniform1i(m_numPointLightsLocation, NumLights);

    for (unsigned int i = 0; i < NumLights; i++) {
        glUniform3f(m_pointLightsLocation[i].Color, pLights[i].Color.x, pLights[i].Color.y, pLights[i].Color.z);
        glUniform1f(m_pointLightsLocation[i].AmbientIntensity, pLights[i].AmbientIntensity);
        glUniform1f(m_pointLightsLocation[i].DiffuseIntensity, pLights[i].DiffuseIntensity);
        glUniform3f(m_pointLightsLocation[i].Position, pLights[i].Position.x, pLights[i].Position.y, pLights[i].Position.z);
        glUniform1f(m_pointLightsLocation[i].Atten.Constant, pLights[i].Attenuation.Constant);
        glUniform1f(m_pointLightsLocation[i].Atten.Linear, pLights[i].Attenuation.Linear);
        glUniform1f(m_pointLightsLocation[i].Atten.Exp, pLights[i].Attenuation.Exp);
    }
}

void LightingTechnique::SetSpotLights(unsigned int NumLights, const SpotLight* pLights)
{
    glUniform1i(m_numSpotLightsLocation, NumLights);

    for (unsigned int i = 0; i < NumLights; i++) {
        glUniform3f(m_spotLightsLocation[i].Color, pLights[i].Color.x, pLights[i].Color.y, pLights[i].Color.z);
        glUniform1f(m_spotLightsLocation[i].AmbientIntensity, pLights[i].AmbientIntensity);
        glUniform1f(m_spotLightsLocation[i].DiffuseIntensity, pLights[i].DiffuseIntensity);
        glUniform3f(m_spotLightsLocation[i].Position, pLights[i].Position.x, pLights[i].Position.y, pLights[i].Position.z);
        Vector3f Direction = pLights[i].Direction;
        Direction.Normalize();
        glUniform3f(m_spotLightsLocation[i].Direction, Direction.x, Direction.y, Direction.z);
        glUniform1f(m_spotLightsLocation[i].Cutoff, cosf(ToRadian(pLights[i].Cutoff)));
        glUniform1f(m_spotLightsLocation[i].Atten.Constant, pLights[i].Attenuation.Constant);
        glUniform1f(m_spotLightsLocation[i].Atten.Linear, pLights[i].Attenuation.Linear);
        glUniform1f(m_spotLightsLocation[i].Atten.Exp, pLights[i].Attenuation.Exp);
    }
}
class ICallbacks
{
public:
    virtual void SpecialKeyboardCB(int Key, int x, int y) = 0;
    virtual void KeyboardCB(unsigned char Key, int x, int y) = 0;
    virtual void PassiveMouseCB(int x, int y) = 0;
    virtual void RenderSceneCB() = 0;
    virtual void IdleCB() = 0;
};
void GLUTBackendInit(int argc, char** argv);

bool GLUTBackendCreateWindow(unsigned int Width, unsigned int Height, unsigned int bpp, bool isFullScreen, const char* pTitle);

void GLUTBackendRun(ICallbacks* pCallbacks);
static ICallbacks* s_pCallbacks = NULL;

static void SpecialKeyboardCB(int Key, int x, int y) {
    s_pCallbacks->SpecialKeyboardCB(Key, x, y);
}

static void KeyboardCB(unsigned char Key, int x, int y) {
    s_pCallbacks->KeyboardCB(Key, x, y);
}

static void PassiveMouseCB(int x, int y) {
    s_pCallbacks->PassiveMouseCB(x, y);
}

static void RenderSceneCB() {
    s_pCallbacks->RenderSceneCB();
}

static void IdleCB() {
    s_pCallbacks->IdleCB();
}

static void InitCallbacks() {
    glutDisplayFunc(RenderSceneCB);
    glutIdleFunc(IdleCB);
    glutSpecialFunc(SpecialKeyboardCB);
    glutPassiveMotionFunc(PassiveMouseCB);
    glutKeyboardFunc(KeyboardCB);
}

void GLUTBackendInit(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}

bool GLUTBackendCreateWindow(unsigned int Width, unsigned int Height, unsigned int bpp, bool isFullScreen, const char* pTitle) {
    if (isFullScreen) {
        char ModeString[64] = { 0 };
        snprintf(ModeString, sizeof(ModeString), "%dx%d@%d", Width, Height, bpp);
        glutGameModeString(ModeString);
        glutEnterGameMode();
    }
    else {
        glutInitWindowSize(Width, Height);
        glutCreateWindow(pTitle);
    }

    GLenum res = glewInit();
    if (res != GLEW_OK) {
        fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
        return false;
    }

    return true;
}

void GLUTBackendRun(ICallbacks* pCallbacks) {
    if (!pCallbacks) {
        fprintf(stderr, "%s : callbacks not specified!\n", __FUNCTION__);
        return;
    }

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glFrontFace(GL_CW);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);

    s_pCallbacks = pCallbacks;
    InitCallbacks();
    glutMainLoop();
}



#define WINDOW_WIDTH  1280
#define WINDOW_HEIGHT 1024

struct Vertex
{
    Vector3f m_pos;
    Vector2f m_tex;
    Vector3f m_normal;

    Vertex() {}

    Vertex(Vector3f pos, Vector2f tex)
    {
        m_pos = pos;
        m_tex = tex;
        m_normal = Vector3f(0.0f, 0.0f, 0.0f);
    }
};

class Main : public ICallbacks
{
public:

    Main()
    {
        m_pGameCamera = NULL;
        m_pTexture = NULL;
        m_pEffect = NULL;
        m_scale = 0.0f;
        m_directionalLight.Color = Vector3f(1.0f, 1.0f, 1.0f);
        m_directionalLight.AmbientIntensity = 0.0f;
        m_directionalLight.DiffuseIntensity = 0.0f;
        m_directionalLight.Direction = Vector3f(1.0f, 0.0f, 0.0f);
    }

    ~Main()
    {
        delete m_pEffect;
        delete m_pGameCamera;
        delete m_pTexture;
    }

    bool Init()
    {
        Vector3f Pos(-10.0f, 0.0f, -10.0f);
        Vector3f Target(1.0f, 0.0f, 1.0f);
        Vector3f Up(0.0, 1.0f, 0.0f);
        m_pGameCamera = new Camera(WINDOW_WIDTH, WINDOW_HEIGHT, Pos, Target, Up);

        unsigned int Indices[] = { 0, 2, 1,
                                   0, 3, 2 };

        CreateIndexBuffer(Indices, sizeof(Indices));

        CreateVertexBuffer(Indices, ARRAY_SIZE_IN_ELEMENTS(Indices));

        m_pEffect = new LightingTechnique();

        if (!m_pEffect->Init())
        {
            printf("Error initializing the lighting technique\n");
            return false;
        }

        m_pEffect->Enable();

        m_pEffect->SetTextureUnit(0);

        m_pTexture = new Texture(GL_TEXTURE_2D, "./Content/test.png");

        if (!m_pTexture->Load()) {
            return false;
        }

        return true;
    }

    void Run()
    {
        GLUTBackendRun(this);
    }

    virtual void RenderSceneCB()
    {
        m_pGameCamera->OnRender();

        glClear(GL_COLOR_BUFFER_BIT);

        m_scale += 0.01f;

        SpotLight sl[2];
        sl[0].DiffuseIntensity = 15.0f;
        sl[0].Color = Vector3f(1.0f, 1.0f, 0.7f);
        sl[0].Position = Vector3f(-0.0f, -1.9f, -0.0f);
        sl[0].Direction = Vector3f(sinf(m_scale), 0.0f, cosf(m_scale));
        sl[0].Attenuation.Linear = 0.1f;
        sl[0].Cutoff = 20.0f;

        sl[1].DiffuseIntensity = 5.0f;
        sl[1].Color = Vector3f(0.0f, 1.0f, 1.0f);
        sl[1].Position = m_pGameCamera->GetPos();
        sl[1].Direction = m_pGameCamera->GetTarget();
        sl[1].Attenuation.Linear = 0.1f;
        sl[1].Cutoff = 10.0f;

        m_pEffect->SetSpotLights(2, sl);


        Pipeline p;
        p.Rotate(0.0f, 0.0f, 0.0f);
        p.WorldPos(0.0f, 0.0f, 1.0f);
        p.SetCamera(m_pGameCamera->GetPos(), m_pGameCamera->GetTarget(), m_pGameCamera->GetUp());
        p.SetPerspectiveProj(60.0f, WINDOW_WIDTH, WINDOW_HEIGHT, 0.1f, 100.0f);
        m_pEffect->SetWVP(p.GetWVPTrans());
        const Matrix4f& WorldTransformation = p.GetWorldTrans();
        m_pEffect->SetWorldMatrix(WorldTransformation);
        m_pEffect->SetDirectionalLight(m_directionalLight);
        m_pEffect->SetEyeWorldPos(m_pGameCamera->GetPos());
        m_pEffect->SetMatSpecularIntensity(1.0f);
        m_pEffect->SetMatSpecularPower(32);

        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)12);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (const GLvoid*)20);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
        m_pTexture->Bind(GL_TEXTURE0);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);

        glutSwapBuffers();
    }

    virtual void IdleCB()
    {
        RenderSceneCB();
    }

    virtual void SpecialKeyboardCB(int Key, int x, int y)
    {
        m_pGameCamera->OnKeyboard(Key);
    }


    virtual void KeyboardCB(unsigned char Key, int x, int y)
    {
        switch (Key) {
        case 'q':
            glutLeaveMainLoop();
            break;

        case 'a':
            m_directionalLight.AmbientIntensity += 0.05f;
            break;

        case 's':
            m_directionalLight.AmbientIntensity -= 0.05f;
            break;

        case 'z':
            m_directionalLight.DiffuseIntensity += 0.05f;
            break;

        case 'x':
            m_directionalLight.DiffuseIntensity -= 0.05f;
            break;
        }
    }


    virtual void PassiveMouseCB(int x, int y)
    {
        m_pGameCamera->OnMouse(x, y);
    }

private:

    void CalcNormals(const unsigned int* pIndices, unsigned int IndexCount,
        Vertex* pVertices, unsigned int VertexCount) {
        for (unsigned int i = 0; i < IndexCount; i += 3) {
            unsigned int Index0 = pIndices[i];
            unsigned int Index1 = pIndices[i + 1];
            unsigned int Index2 = pIndices[i + 2];
            Vector3f v1 = pVertices[Index1].m_pos - pVertices[Index0].m_pos;
            Vector3f v2 = pVertices[Index2].m_pos - pVertices[Index0].m_pos;
            Vector3f Normal = v1.Cross(v2);
            Normal.Normalize();

            pVertices[Index0].m_normal += Normal;
            pVertices[Index1].m_normal += Normal;
            pVertices[Index2].m_normal += Normal;
        }

        for (unsigned int i = 0; i < VertexCount; i++) {
            pVertices[i].m_normal.Normalize();
        }
    }


    void CreateVertexBuffer(const unsigned int* pIndices, unsigned int IndexCount)
    {
        Vertex Vertices[4] = { Vertex(Vector3f(-10.0f, -2.0f, -10.0f), Vector2f(0.0f, 0.0f)),
                               Vertex(Vector3f(10.0f, -2.0f, -10.0f), Vector2f(1.0f, 0.0f)),
                               Vertex(Vector3f(10.0f, -2.0f, 10.0f), Vector2f(1.0f, 1.0f)),
                               Vertex(Vector3f(-10.0f, -2.0f, 10.0f), Vector2f(0.0f, 1.0f)) };

        unsigned int VertexCount = ARRAY_SIZE_IN_ELEMENTS(Vertices);

        CalcNormals(pIndices, IndexCount, Vertices, VertexCount);

        glGenBuffers(1, &m_VBO);
        glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
    }

    void CreateIndexBuffer(const unsigned int* pIndices, unsigned int SizeInBytes)
    {
        glGenBuffers(1, &m_IBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, SizeInBytes, pIndices, GL_STATIC_DRAW);
    }


    GLuint m_VBO;
    GLuint m_IBO;
    LightingTechnique* m_pEffect;
    Texture* m_pTexture;
    Camera* m_pGameCamera;
    float m_scale;
    DirectionalLight m_directionalLight;
};


int main(int argc, char** argv)
{
    Magick::InitializeMagick(nullptr);
    GLUTBackendInit(argc, argv);

    if (!GLUTBackendCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, 32, false, "OpenGL tutors")) {
        return 1;
    }

    Main* pApp = new Main();

    if (!pApp->Init()) {
        return 1;
    }

    pApp->Run();

    delete pApp;

    return 0;
}