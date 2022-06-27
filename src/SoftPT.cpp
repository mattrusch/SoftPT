// SoftPT.cpp

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <vector>
#include <cassert>

const int USE_SKY_COLOR = 0;

const float kPi = 3.1415927f;
const float kEpsilon = 0.00001f;

class Vector3
{
public:
    Vector3() = default;
    Vector3(const Vector3& rhs) = default;
    explicit Vector3(float in)
        : x(in)
        , y(in)
        , z(in)
    {}
    Vector3(float inX, float inY, float inZ)
        : x(inX)
        , y(inY)
        , z(inZ)
    {}

    float Dot(const Vector3& rhs) const
    {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vector3 Cross(const Vector3& rhs) const
    {
        return Vector3{y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }

    float Length() const
    {
        return sqrtf(Dot(*this));
    }

    Vector3 operator-(const Vector3& rhs) const
    {
        return { x - rhs.x, y - rhs.y, z - rhs.z };
    }

    Vector3 operator+(const Vector3& rhs) const
    {
        return { x + rhs.x, y + rhs.y, z + rhs.z };
    }

    Vector3 operator*(const Vector3& rhs) const
    {
        return { x * rhs.x, y * rhs.y, z * rhs.z };
    }

    Vector3 operator*(float rhs) const
    {
        return { x * rhs, y * rhs, z * rhs };
    }

    Vector3 operator+(float rhs) const
    {
        return { x + rhs, y + rhs, z + rhs };
    }

    Vector3 Normalize() const
    {
        return *this * (1.0f / Length());
    }

    float Distance(const Vector3& rhs) const
    {
        return (*this - rhs).Length();
    }

    bool IsEquivalent(const Vector3& rhs, const float maxDelta = kEpsilon) const
    {
        Vector3 delta = rhs - *this;
        return delta.Length() < maxDelta;
    }

    float x;
    float y;
    float z;
};

class Ray
{
public:
    Vector3 origin;
    Vector3 direction;
};

class Material
{
public:
    Vector3 albedo;
    Vector3 emissive;
    float   roughness;
};

class Sphere
{
public:
    Vector3         center;
    float           radius;
    const Material* material;
};

template< typename T >
T Lerp(T val0, T val1, float t)
{
    return val0 + (val1 - val0) * t;
}

float Saturate(float in)
{
    return in < 0.0f ? 0.0f : in > 1.0f ? 1.0f : in;
}

float Max(float a, float b)
{
    return a > b ? a : b;
}

std::vector<Vector3> Intersect(const Ray& ray, const Sphere& sphere)
{
    // Calculate discriminant
    Vector3 rayOriginToSphereCenter = ray.origin - sphere.center;
    float a = ray.direction.Dot(ray.direction);
    float b = 2.0f * ray.direction.Dot(rayOriginToSphereCenter);
    float c = rayOriginToSphereCenter.Dot(rayOriginToSphereCenter) - sphere.radius * sphere.radius;
    float discriminant = b * b - 4.0f * a * c;

    std::vector<Vector3> result;
    if (discriminant < 0.0f)
    {
        return result;
    }
    else
    {
        // Solve quadratic for real roots
        float t0 = (-1.0f * b + sqrtf(discriminant)) / (2.0f * a);
        if (t0 >= 0.0f)
        {
            result.emplace_back(ray.origin + ray.direction * t0);
        }

        if (discriminant > kEpsilon)
        {
            float t1 = (-1.0f * b - sqrtf(discriminant)) / (2.0f * a);
            if (t1 >= 0.0f)
            {
                result.emplace_back(ray.origin + ray.direction * t1);

                // Order by distance; smallest positive root is closest
                if ((t0 >= 0.0f && t1 >= 0.0f) && (t1 < t0))
                {
                    std::swap(result[0], result[1]);
                }
            }
        }

        return result;
    }
}

void RandomTangentFrame(const Vector3& normal, Vector3& outTangent, Vector3& outBitangent)
{
    const Vector3 right{ -1.0f, 0.0f, 0.0f };
    const Vector3 up{ 0.0f, 1.0f, 0.0f };

    outTangent = normal.IsEquivalent(right) ? up : right;
    outBitangent = normal.Cross(outTangent).Normalize();
    outTangent = outBitangent.Cross(normal).Normalize();
}

Vector3 RandomVector(const Vector3& normal, float rand0, float rand1)
{
    // Random direction over hemisphere centered on (0, 1, 0)
    // (x, y, z) = (sqrt(1 - rand0^2)*cos(2*pi*rand1), rand0, sqrt(1 - rand0^2)*sin(2*pi*rand1))
    float sqrtFactor = sqrtf(1.0f - rand0 * rand0);
    float cosFactor = cosf(2.0f * kPi * rand1);
    float sinFactor = sinf(2.0f * kPi * rand1);
    Vector3 randVec = Vector3{ sqrtFactor * cosFactor, rand0, sqrtFactor * sinFactor };

    // Transform to 'arbitrary' tangent frame centered around normal
    Vector3 tangent;
    Vector3 bitangent;
    RandomTangentFrame(normal, tangent, bitangent);
    Vector3 rowX{ tangent.x, normal.x, bitangent.x };
    Vector3 rowY{ tangent.y, normal.y, bitangent.y };
    Vector3 rowZ{ tangent.z, normal.z, bitangent.z };

    Vector3 result{ randVec.Dot(rowX), randVec.Dot(rowY), randVec.Dot(rowZ) };
    return result;
}

const int kMaxBounces = 6;
Vector3 TracePath(const Ray& ray, const std::vector<Sphere>& spheres, int bounce)
{
    if (bounce == kMaxBounces)
    {
        return Vector3{ 0.0f, 0.0f, 0.0f };
    }

    int nearestSphereIndex = INT_MAX;
    float nearestDistance = FLT_MAX;
    Vector3 nearestIntersection(FLT_MAX);

    for (int i = 0; i < spheres.size(); ++i)
    {
        auto intersection = Intersect(ray, spheres[i]);
        if (!intersection.empty())
        {
            float distance = (intersection[0] - ray.origin).Length();
            if (distance < nearestDistance)
            {
                nearestSphereIndex = i;
                nearestIntersection = intersection[0];
                nearestDistance = distance;
            }
        }
    }

    if (nearestSphereIndex == INT_MAX)
    {
        #if USE_SKY_COLOR 1
        return Lerp(Vector3{ 0.0f, 0.0f, 0.0f }, Vector3{ 0.25f, 0.55f, 0.75f }, ray.direction.y);
        #else
        return Vector3{ 0.0f, 0.0f, 0.0f };
        #endif//USE_SKY_COLOR
    }
    else
    {
        const Sphere& closestSphere = spheres[nearestSphereIndex];
        Vector3 normal = (nearestIntersection - closestSphere.center).Normalize();

        float rand0 = (float)(rand()) / (float)(RAND_MAX + 1);
        float rand1 = (float)(rand()) / (float)(RAND_MAX + 1);
        Vector3 newDir = RandomVector(normal, rand0, rand1);
        
        float check = newDir.Dot(normal);
        assert(check >= (0.0f - kEpsilon));

        Ray newRay{ nearestIntersection + normal * kEpsilon, newDir };
        return closestSphere.material->emissive + closestSphere.material->albedo * TracePath(newRay, spheres, ++bounce) * normal.Dot(newDir);
    }
}

// MSR_TODO: This is horrid, fix
#define SPHERE_CENTER_RAD(x, y, z) Vector3{x, y, z}, (Vector3{x, y, z} - globalCenter).Length() - globalRadius

void InitScene(std::vector<Sphere>& spheres, std::vector<Material>& materials)
{
    materials.emplace_back(Material{ Vector3{1.0f, 1.0f, 1.0f}, Vector3{0.0f, 0.0f, 0.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{0.5f, 1.0f, 0.5f}, Vector3{10.0f, 10.0f, 10.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{1.0f, 0.5f, 0.5f}, Vector3{0.0f, 0.0f, 0.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{0.5f, 0.5f, 1.0f}, Vector3{0.0f, 0.0f, 0.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{0.5f, 1.0f, 0.75f}, Vector3{0.0f, 0.0f, 0.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{1.0f, 1.0f, 0.5f}, Vector3{10.0f, 5.0f, 5.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{1.0f, 1.0f, 1.0f}, Vector3{0.0f, 0.0f, 0.0f}, {1.0f} });
    materials.emplace_back(Material{ Vector3{0.5f, 1.0f, 1.0f}, Vector3{5.0f, 5.0f, 10.0f}, {1.0f} });

    float globalRadius = 100.0f;
    Vector3 globalCenter{ 0.0f, -globalRadius, 0.0f };
    spheres.emplace_back(Sphere{ globalCenter, {100.0f}, {&materials[0]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(0.0f, 0.125f, 0.0f),    {&materials[1]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(-0.5f, 0.125f, 0.0f),   {&materials[2]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(0.5f, 0.25f, 0.5f),     {&materials[3]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(0.25f, 0.05f, -0.25f),  {&materials[4]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(-0.25f, 0.5f, 1.5f),    {&materials[5]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(0.25f, 0.1f, 0.25f),    {&materials[6]} });
    spheres.emplace_back(Sphere{ SPHERE_CENTER_RAD(-0.65f, 0.05f, -0.25f), {&materials[7]} });
}

void Render(int width, int height, HDC hdc)
{
    std::vector<Sphere> spheres;
    std::vector<Material> materials;

    InitScene(spheres, materials);

    Vector3 camTarget(0.0f, 0.0f, 0.0f);
    Vector3 camPos{ 0.0f, 0.5f, -1.0f };
    Vector3 camUp{ 0.0f, 1.0f, 0.0f };
    Vector3 camRight = camUp.Cross((camTarget - camPos).Normalize());
    camUp = camRight.Cross(camPos.Normalize());

    float dx = 2.0f / static_cast<float>(width);
    float dy = 2.0f / static_cast<float>(height);

    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            Ray ray;
            ray.origin = camPos;

            Vector3 nearPlanePos;
            nearPlanePos = camRight * (-1.0f + dx * static_cast<float>(i)) + camUp * (1.0f - dy * static_cast<float>(j));
            ray.direction = (nearPlanePos - camPos).Normalize();

            const int kNumSamples = 4;
            Vector3 colorSum{ 0.0f };
            for (int s = 0; s < kNumSamples; ++s)
            {
                colorSum = colorSum + TracePath(ray, spheres, 0);
            }

            Vector3 color = colorSum * Vector3{ 1.0f / (float)kNumSamples };

            COLORREF fragmentColor = RGB(static_cast<BYTE>(Saturate(color.x) * 255.0f), static_cast<BYTE>(Saturate(color.y) * 255.0f), static_cast<BYTE>(Saturate(color.z) * 255.0f));
            SetPixel(hdc, i, j, fragmentColor);
        }
    }
}

// Butchered win32 boilerplate appwizard code follows:
HINSTANCE hInst;
CHAR* szWindowClass = "SoftPT";

// Forward declarations of functions included in this code module:
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
    _In_opt_ HINSTANCE hPrevInstance,
    _In_ LPWSTR    lpCmdLine,
    _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // Initialize global strings
    MyRegisterClass(hInstance);

    // Perform application initialization:
    if (!InitInstance(hInstance, nCmdShow))
    {
        return FALSE;
    }

    MSG msg;

    // Main message loop:
    while (GetMessage(&msg, nullptr, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return (int)msg.wParam;
}

ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEX wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc = WndProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(hInstance, IDI_APPLICATION);
    wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wcex.lpszMenuName = NULL;
    wcex.lpszClassName = szWindowClass;
    wcex.hIconSm = NULL;

    return RegisterClassEx(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    hInst = hInstance; // Store instance handle in our global variable

    HWND hWnd = CreateWindow(szWindowClass, "SoftPT", WS_OVERLAPPEDWINDOW,
        100, 100, 1024, 1024, nullptr, nullptr, hInstance, nullptr);

    if (!hWnd)
    {
        return FALSE;
    }

    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);

    return TRUE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hWnd, &ps);
        // TODO: Add any drawing code that uses hdc here...
        RECT winRect;
        GetWindowRect(hWnd, &winRect);
        Render(winRect.right - winRect.left, winRect.bottom - winRect.top, hdc);
        EndPaint(hWnd, &ps);
    }
    break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}
