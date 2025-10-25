// main.cpp
// Build: see CMake section below
// Deps: Hello ImGui (GLFW + OpenGL3 backend), OpenCascade (OCCT)

#include <hello_imgui/hello_imgui.h>
#include <imgui.h>

#include <vector>
#include <string>
#include <optional>
#include <cmath>
#include <cstdio>

// ---- OpenGL minimal helpers (no external GL loader needed: Hello ImGui sets one up) ----
#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glad/glad.h>
#endif

// ---- OpenCascade (OCCT) includes for STEP reading + meshing ----
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <gp_Pnt.hxx>
#include <TopLoc_Location.hxx>

// ---------------- Mesh container ----------------
struct Mesh {
    std::vector<float> vertices;   // interleaved pos + normal: [px,py,pz, nx,ny,nz]...
    std::vector<unsigned int> indices;
    GLuint vao=0, vbo=0, ebo=0;
    bool ready = false;

    void upload() {
        if (ready) return;
        glGenVertexArrays(1,&vao);
        glBindVertexArray(vao);
        glGenBuffers(1,&vbo);
        glGenBuffers(1,&ebo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
        GLsizei stride = 6 * sizeof(float);
        glEnableVertexAttribArray(0); // position
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,stride,(void*)0);
        glEnableVertexAttribArray(1); // normal
        glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,stride,(void*)(3*sizeof(float)));
        glBindVertexArray(0);
        ready = true;
    }
    void draw() const {
        if (!ready) return;
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, (GLsizei)indices.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
    void clearGL() {
        if (!ready) return;
        glDeleteBuffers(1,&vbo);
        glDeleteBuffers(1,&ebo);
        glDeleteVertexArrays(1,&vao);
        vao=vbo=ebo=0; ready=false;
    }
};

// ------------- Simple shader -------------------
static const char* kVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vNrm;
out vec3 vPosW;
void main(){
    vec4 pw = uModel * vec4(aPos,1.0);
    vPosW = pw.xyz;
    vNrm  = mat3(uModel) * aNrm;
    gl_Position = uMVP * vec4(aPos,1.0);
}
)";

static const char* kFS = R"(#version 330 core
in vec3 vNrm;
in vec3 vPosW;
out vec4 FragColor;
uniform vec3 uEye;
uniform vec3 uColor;
void main(){
    vec3 N = normalize(vNrm);
    vec3 L = normalize(vec3(0.4, 0.7, 1.0));
    float diff = max(dot(N,L), 0.1);
    vec3 base = uColor * diff;
    // simple spec
    vec3 V = normalize(uEye - vPosW);
    vec3 H = normalize(L+V);
    float spec = pow(max(dot(N,H),0.0), 32.0) * 0.25;
    FragColor = vec4(base + spec, 1.0);
}
)";

GLuint makeProgram(const char* vs, const char* fs) {
    auto compile = [](GLenum type, const char* src)->GLuint{
        GLuint s = glCreateShader(type);
        glShaderSource(s,1,&src,nullptr);
        glCompileShader(s);
        GLint ok=0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if(!ok){ char log[2048]; glGetShaderInfoLog(s,2048,nullptr,log); fprintf(stderr,"Shader error: %s\n",log); }
        return s;
    };
    GLuint v = compile(GL_VERTEX_SHADER, vs);
    GLuint f = compile(GL_FRAGMENT_SHADER, fs);
    GLuint p = glCreateProgram();
    glAttachShader(p,v); glAttachShader(p,f);
    glLinkProgram(p);
    GLint ok=0; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if(!ok){ char log[2048]; glGetProgramInfoLog(p,2048,nullptr,log); fprintf(stderr,"Link error: %s\n",log); }
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

// ----------- Math helpers -------------
struct Mat4 { float m[16]; };
Mat4 mul(const Mat4& A, const Mat4& B){
    Mat4 R{};
    for(int r=0;r<4;r++) for(int c=0;c<4;c++){
        R.m[r*4+c]=0.f;
        for(int k=0;k<4;k++) R.m[r*4+c]+=A.m[r*4+k]*B.m[k*4+c];
    }
    return R;
}
Mat4 identity(){ Mat4 M{}; for(int i=0;i<16;i++) M.m[i]= (i%5==0)?1.f:0.f; return M; }
Mat4 translate(float x,float y,float z){ Mat4 M=identity(); M.m[12]=x; M.m[13]=y; M.m[14]=z; return M; }
Mat4 perspective(float fovyRad, float aspect, float zn, float zf){
    float f=1.f/std::tan(fovyRad*0.5f);
    Mat4 M{};
    M.m[0]=f/aspect; M.m[5]=f; M.m[10]=(zf+zn)/(zn-zf); M.m[11]=-1.f; M.m[14]=(2*zf*zn)/(zn-zf);
    return M;
}
Mat4 lookAt(const float eye[3], const float center[3], const float up[3]){
    auto norm = [](float v[3]){ float s=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); if(s>0){v[0]/=s;v[1]/=s;v[2]/=s;}};
    float F[3] = {center[0]-eye[0], center[1]-eye[1], center[2]-eye[2]}; norm(F);
    float U[3] = {up[0],up[1],up[2]}; norm(U);
    float S[3] = {F[1]*U[2]-F[2]*U[1], F[2]*U[0]-F[0]*U[2], F[0]*U[1]-F[1]*U[0]}; norm(S);
    float R[3] = {S[1]*F[2]-S[2]*F[1], S[2]*F[0]-S[0]*F[2], S[0]*F[1]-S[1]*F[0]};
    Mat4 M=identity();
    M.m[0]=S[0]; M.m[4]=S[1]; M.m[8]=S[2];
    M.m[1]=R[0]; M.m[5]=R[1]; M.m[9]=R[2];
    M.m[2]=-F[0];M.m[6]=-F[1];M.m[10]=-F[2];
    Mat4 T=translate(-eye[0],-eye[1],-eye[2]);
    return mul(M,T);
}
// rotation: align +Z to given unit direction D (Rodrigues)
Mat4 alignZtoD(const float Dunit[3]){
    float z[3]={0,0,1};
    float v[3]={z[1]*Dunit[2]-z[2]*Dunit[1], z[2]*Dunit[0]-z[0]*Dunit[2], z[0]*Dunit[1]-z[1]*Dunit[0]};
    float s = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    float c = z[0]*Dunit[0]+z[1]*Dunit[1]+z[2]*Dunit[2];
    Mat4 R=identity();
    if (s<1e-6f){ // parallel or anti-parallel
        if (c>0.9999f) return R; // identity
        // 180deg around X
        R.m[5]=-1; R.m[10]=-1; return R;
    }
    float vx=v[0], vy=v[1], vz=v[2];
    float K[16]={
        0, -vz, vy, 0,
        vz, 0, -vx,0,
        -vy, vx, 0,0,
        0,0,0,0
    };
    // R = I + K + K^2*(1-c)/s^2
    // compute K^2
    float K2[16]={0};
    auto get=[&](int r,int c){return K[r*4+c];};
    auto set=[&](float* M,int r,int c,float val){M[r*4+c]=val;};
    for(int r=0;r<4;r++) for(int c2=0;c2<4;c2++){
        float sum=0; for(int k=0;k<4;k++) sum+=get(r,k)*get(k,c2);
        set(K2,r,c2,sum);
    }
    float a = (1.f - c) / (s*s);
    for(int i=0;i<16;i++) R.m[i]+=K[i] + a*K2[i];
    return R;
}

// -------------- STEP -> Mesh via OCCT ----------------
static Mesh loadStepMesh(const std::string& path, double deflection = 0.5, double angle = 0.8)
{
    Mesh out;
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(path.c_str());
    if (stat != IFSelect_RetDone) {
        fprintf(stderr, "Failed to read STEP: %s\n", path.c_str());
        return out;
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();

    // tessellate
    BRepMesh_IncrementalMesh mesher(shape, deflection, false, angle, true);

    // gather triangles
    TopExp_Explorer exp;
    for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next())
    {
        TopoDS_Face face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) poly = BRep_Tool::Triangulation(face, loc);
        if (poly.IsNull()) continue;

        gp_Trsf tr = loc.Transformation();
        const TColgp_Array1OfPnt& nodes = poly->Nodes();
        const Poly_Array1OfTriangle& tris = poly->Triangles();

        // per-face normal (fallback)
        // accumulate per-vertex normals
        std::vector<int> firstIndex(nodes.Length()+1, -1);
        size_t baseVertex = out.vertices.size()/6;
        // push positions (normals later)
        for (int i = nodes.Lower(); i <= nodes.Upper(); ++i) {
            gp_Pnt p = nodes(i).Transformed(tr);
            out.vertices.push_back((float)p.X());
            out.vertices.push_back((float)p.Y());
            out.vertices.push_back((float)p.Z());
            out.vertices.push_back(0.f); // nx placeholder
            out.vertices.push_back(0.f); // ny
            out.vertices.push_back(0.f); // nz
        }

        auto addTri = [&](int i0,int i1,int i2){
            unsigned int a = (unsigned int)(baseVertex + (i0-1));
            unsigned int b = (unsigned int)(baseVertex + (i1-1));
            unsigned int c = (unsigned int)(baseVertex + (i2-1));
            out.indices.push_back(a); out.indices.push_back(b); out.indices.push_back(c);

            // compute face normal
            auto getPos=[&](unsigned int idx){
                size_t k = idx*6;
                return std::array<float,3>{ out.vertices[k+0], out.vertices[k+1], out.vertices[k+2] };
            };
            auto pa=getPos(a), pb=getPos(b), pc=getPos(c);
            float ux=pb[0]-pa[0], uy=pb[1]-pa[1], uz=pb[2]-pa[2];
            float vx=pc[0]-pa[0], vy=pc[1]-pa[1], vz=pc[2]-pa[2];
            float nx=uy*vz-uz*vy, ny=uz*vx-ux*vz, nz=ux*vy-uy*vx;
            // accumulate to vertex normals
            for (unsigned int vi: {a,b,c}) {
                size_t kk=vi*6+3;
                out.vertices[kk+0]+=nx; out.vertices[kk+1]+=ny; out.vertices[kk+2]+=nz;
            }
        };

        for (int t = tris.Lower(); t <= tris.Upper(); ++t) {
            int i0,i1,i2; tris(t).Get(i0,i1,i2);
            addTri(i0,i1,i2);
        }
    }

    // normalize normals
    for (size_t i=0;i<out.vertices.size(); i+=6) {
        float nx=out.vertices[i+3], ny=out.vertices[i+4], nz=out.vertices[i+5];
        float s=std::sqrt(nx*nx+ny*ny+nz*nz);
        if (s>1e-12f){ out.vertices[i+3]=nx/s; out.vertices[i+4]=ny/s; out.vertices[i+5]=nz/s; }
        else { out.vertices[i+3]=0; out.vertices[i+4]=0; out.vertices[i+5]=1; }
    }

    return out;
}

// ---------------- App State ----------------
struct AppState {
    std::string sensorStepPath = "";
    std::string magnetStepPath = "";
    Mesh sensorMesh, magnetMesh;

    // magnet pose
    float pos[3] = {0.f, 0.f, 5.f}; // default a bit in front
    float dir[3] = {0.f, 0.f, 1.f}; // axis direction (normalized inside)
    bool  autoNormalize = true;

    // view
    float camDist = 20.f;
    float yaw = 0.6f, pitch = 0.3f;
    bool  showGrid = true;

    GLuint shader=0;
};

// simple grid
void drawGrid(GLuint shader, const Mat4& VP){
    std::vector<float> v;
    const int N=20; const float step=1.0f;
    for(int i=-N;i<=N;i++){
        v.push_back(i*step); v.push_back(0); v.push_back(-N*step);
        v.push_back(i*step); v.push_back(0); v.push_back(N*step);
        v.push_back(-N*step); v.push_back(0); v.push_back(i*step);
        v.push_back(N*step);  v.push_back(0); v.push_back(i*step);
    }
    GLuint vao,vbo; glGenVertexArrays(1,&vao); glGenBuffers(1,&vbo);
    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(float), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glUseProgram(shader);
    GLint uMVP = glGetUniformLocation(shader,"uMVP");
    GLint uModel = glGetUniformLocation(shader,"uModel");
    GLint uColor = glGetUniformLocation(shader,"uColor");
    glUniform3f(uColor, 0.5f,0.5f,0.5f);
    Mat4 I = identity();
    glUniformMatrix4fv(uModel,1,GL_FALSE,I.m);
    glUniformMatrix4fv(uMVP,1,GL_FALSE,VP.m);
    glDrawArrays(GL_LINES, 0, (GLsizei)(v.size()/3));
    glBindVertexArray(0);
    glDeleteBuffers(1,&vbo);
    glDeleteVertexArrays(1,&vao);
}

// ---------------- GUI ----------------
void CommandGui(AppState& s) {
    ImGui::Text("STEP paths");
    ImGui::InputText("Sensor board STEP", &s.sensorStepPath);
    ImGui::SameLine();
    if (ImGui::Button("Load sensor")) {
        s.sensorMesh.clearGL();
        s.sensorMesh = loadStepMesh(s.sensorStepPath);
        s.sensorMesh.upload();
    }
    ImGui::InputText("Magnet STEP", &s.magnetStepPath);
    ImGui::SameLine();
    if (ImGui::Button("Load magnet")) {
        s.magnetMesh.clearGL();
        s.magnetMesh = loadStepMesh(s.magnetStepPath);
        s.magnetMesh.upload();
    }

    ImGui::Separator();
    ImGui::Text("Magnet pose (axis aligns to dir)");
    ImGui::InputFloat3("Position (x,y,z)", s.pos);
    if (ImGui::Checkbox("Normalize dir", &s.autoNormalize)) { /* noop */ }
    if (ImGui::InputFloat3("Direction (dx,dy,dz)", s.dir) && s.autoNormalize) {
        float l = std::sqrt(s.dir[0]*s.dir[0]+s.dir[1]*s.dir[1]+s.dir[2]*s.dir[2]);
        if (l>1e-9f){ s.dir[0]/=l; s.dir[1]/=l; s.dir[2]/=l; }
    }

    ImGui::Separator();
    ImGui::Text("View");
    ImGui::SliderFloat("Distance", &s.camDist, 5.f, 100.f);
    ImGui::SliderFloat("Yaw",   &s.yaw,  -3.14f, 3.14f);
    ImGui::SliderFloat("Pitch", &s.pitch,-1.5f,  1.5f);
    ImGui::Checkbox("Show grid", &s.showGrid);

    ImGui::Separator();
    if (ImGui::Button("Quit")) {
        HelloImGui::GetRunnerParams()->appShallExit = true; // exit.  [oai_citation:2‡hello_imgui_manual.pdf](file-service://file-Pvpirxmy4FUKy95Twbqmii)
    }
}

// ---------------- 3D render in CustomBackground ----------------
void RenderScene(AppState& s) {
    // init GL state & shader once
    if (s.shader==0) {
        s.shader = makeProgram(kVS, kFS);
        glEnable(GL_DEPTH_TEST);
    }

    // viewport from framebuffer size
    ImVec2 fb = ImGui::GetIO().DisplaySize;
    glViewport(0,0,(GLsizei)fb.x,(GLsizei)fb.y);
    glClearColor(0.08f,0.09f,0.11f,1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // camera
    float eye[3] = {
        s.camDist*std::cos(s.pitch)*std::cos(s.yaw),
        s.camDist*std::sin(s.pitch),
        s.camDist*std::cos(s.pitch)*std::sin(s.yaw)
    };
    float center[3]={0,0,0};
    float up[3]={0,1,0};
    Mat4 V = lookAt(eye, center, up);
    float aspect = (fb.y>0)? (fb.x/fb.y) : 1.f;
    Mat4 P = perspective(45.f*(3.14159f/180.f), aspect, 0.1f, 1000.f);
    Mat4 VP = mul(P,V);

    glUseProgram(s.shader);
    GLint uMVP   = glGetUniformLocation(s.shader, "uMVP");
    GLint uModel = glGetUniformLocation(s.shader, "uModel");
    GLint uEye   = glGetUniformLocation(s.shader, "uEye");
    GLint uColor = glGetUniformLocation(s.shader, "uColor");
    glUniform3f(uEye, eye[0], eye[1], eye[2]);

    if (s.showGrid) drawGrid(s.shader, VP);

    // Sensor board at origin
    if (s.sensorMesh.ready) {
        Mat4 M = identity();
        Mat4 MVP = VP; // since shader multiplies uMVP * vec4(aPos,1) in view space? (we used model=identity for position/normal)
        glUniformMatrix4fv(uModel,1,GL_FALSE,M.m);
        glUniformMatrix4fv(uMVP,1,GL_FALSE,MVP.m);
        glUniform3f(uColor, 0.1f, 0.6f, 0.9f);
        s.sensorMesh.draw();
    }

    // Magnet with position + direction
    if (s.magnetMesh.ready) {
        float dir[3] = {s.dir[0], s.dir[1], s.dir[2]};
        float l = std::sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]);
        if (l<1e-9f) { dir[2]=1.f; dir[0]=dir[1]=0.f; } else if (s.autoNormalize) { dir[0]/=l; dir[1]/=l; dir[2]/=l; }
        Mat4 R = alignZtoD(dir);
        Mat4 T = translate(s.pos[0], s.pos[1], s.pos[2]);
        Mat4 M = mul(T,R);
        Mat4 MVP = mul(VP, M);
        glUniformMatrix4fv(uModel,1,GL_FALSE,M.m);
        glUniformMatrix4fv(uMVP,1,GL_FALSE,MVP.m);
        glUniform3f(uColor, 0.9f, 0.25f, 0.1f);
        s.magnetMesh.draw();
    }
}

int main(int, char**) {
    static AppState app;

    HelloImGui::RunnerParams params;
    params.appWindowParams.windowTitle = "Sensor + Magnet (STEP) — Hello ImGui";
    params.imGuiWindowParams.showMenuBar = false;
    params.imGuiWindowParams.defaultImGuiWindowType =
        HelloImGui::DefaultImGuiWindowType::ProvideFullScreenDockSpace; // full screen dockspace.  [oai_citation:3‡hello_imgui_manual.pdf](file-service://file-Pvpirxmy4FUKy95Twbqmii)

    // Docking layout: left = Commands panel, center = 3D view background.
    HelloImGui::DockingSplit left;
    left.initialDock = "MainDockSpace";
    left.newDock = "CommandSpace";
    left.direction = ImGuiDir_Left;
    left.ratio = 0.28f;

    params.dockingParams.dockingSplits = {left};
    HelloImGui::DockableWindow commands;
    commands.label = "Commands";
    commands.dockSpaceName = "CommandSpace";
    commands.GuiFunction = [&]{ CommandGui(app); };
    params.dockingParams.dockableWindows = { commands }; // define dockable windows.  [oai_citation:4‡hello_imgui_manual.pdf](file-service://file-Pvpirxmy4FUKy95Twbqmii)

    // Render 3D scene behind GUI
    params.callbacks.CustomBackground = [&](){ RenderScene(app); }; // draw 3D under GUI.  [oai_citation:5‡hello_imgui_manual.pdf](file-service://file-Pvpirxmy4FUKy95Twbqmii)

    HelloImGui::Run(params); // run app loop.  [oai_citation:6‡hello_imgui_manual.pdf](file-service://file-Pvpirxmy4FUKy95Twbqmii)

    // cleanup (GL objects will be destroyed by OS at exit, but tidy up anyway)
    app.sensorMesh.clearGL();
    app.magnetMesh.clearGL();

    return 0;
}