#include <glad/glad.h>

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <iostream>
#include <vector>

#include "hello_imgui/hello_imgui.h"
#include "imgui.h"

//--------------------------------------------------------------------
// Tessellate the TopoDS_Shape and collect triangle vertices
//--------------------------------------------------------------------
static std::vector<float> Tessellate(const TopoDS_Shape& shape) {
	BRepMesh_IncrementalMesh mesher(shape, 0.5, false, 0.5, false);
	std::vector<float> verts;

	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
		const TopoDS_Face& face = TopoDS::Face(exp.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull()) continue;

		gp_Trsf trsf = loc.Transformation();
		const int nTri = tri->NbTriangles();
		for (int i = 1; i <= nTri; ++i) {
			Poly_Triangle t = tri->Triangle(i);
			int n1, n2, n3;
			t.Get(n1, n2, n3);
			gp_Pnt p1 = tri->Node(n1).Transformed(trsf);
			gp_Pnt p2 = tri->Node(n2).Transformed(trsf);
			gp_Pnt p3 = tri->Node(n3).Transformed(trsf);
			verts.insert(verts.end(), {(float)p1.X(), (float)p1.Y(), (float)p1.Z(), (float)p2.X(), (float)p2.Y(), (float)p2.Z(), (float)p3.X(), (float)p3.Y(), (float)p3.Z()});
		}
	}
	std::cout << "Triangles: " << verts.size() / 9 << std::endl;
	return verts;
}

//--------------------------------------------------------------------
// Simple OpenGL mesh class
//--------------------------------------------------------------------
struct GLMesh {
	GLuint vao = 0, vbo = 0;
	GLsizei count = 0;

	GLMesh(const std::vector<float>& v) {
		count = (GLsizei)(v.size() / 3);
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);
		glBindVertexArray(vao);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glBindVertexArray(0);
	}
	~GLMesh() {
		glDeleteBuffers(1, &vbo);
		glDeleteVertexArrays(1, &vao);
	}

	void Draw() const {
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.1f, 0.12f, 0.15f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBindVertexArray(vao);
		glDrawArrays(GL_TRIANGLES, 0, count);
		glBindVertexArray(0);
	}
};

//--------------------------------------------------------------------
// STEP loader using the standard high-level reader
//--------------------------------------------------------------------
static TopoDS_Shape LoadSTEP(const std::string& path) {
	STEPControl_Reader reader;
	if (reader.ReadFile(path.c_str()) != IFSelect_RetDone) {
		std::cerr << "Cannot read " << path << std::endl;
		return TopoDS_Shape();
	}
	reader.TransferRoots();
	return reader.OneShape();
}

//--------------------------------------------------------------------
// Main application logic
//--------------------------------------------------------------------
int main(int argc, char** argv) {
	std::filesystem::path stepFile = std::filesystem::path(CMAKE_SOURCE_DIR) / "data" / "3D_PCB_MAGNETOMETER_ARRAY_V1_1.step";

	HelloImGui::RunnerParams params;
	params.appWindowParams.windowTitle = "Minimal STEP Viewer";
	params.appWindowParams.windowGeometry.size = {1024, 720};
	params.rendererBackendType = HelloImGui::RendererBackendType::OpenGL3;

	std::shared_ptr<GLMesh> mesh = nullptr;
	params.callbacks.PostInit = [&]() {
		TopoDS_Shape shape = LoadSTEP(stepFile);
		std::vector<float> verts = Tessellate(shape);
		mesh = std::make_shared<GLMesh>(verts);
	};

	params.callbacks.ShowGui = [=]() {
		ImGui::Begin("STEP Viewer");
		ImGui::Text("File: %s", stepFile.c_str());
		if (mesh)
			ImGui::Text("Triangles: %d", mesh->count / 3);
		else
			ImGui::Text("Loading...");
		ImGui::End();

		ImGui::Begin("3D View");
		ImGui::BeginChild("RenderRegion", ImVec2(800, 600), true);
		mesh->Draw();
		ImGui::EndChild();
		ImGui::End();
	};

	HelloImGui::Run(params);
	return 0;
}