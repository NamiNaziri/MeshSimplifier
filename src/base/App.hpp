#pragma once

#include "gui/Window.hpp"
#include "gui/CommonControls.hpp"
#include "VirtualTrackballCamera.h"
#include <string>
#include <vector>

#include "../..//QEM_SurfaceSimplification.h"

namespace FW {

	struct Vertex
	{
		Vec3f position;
		Vec3f normal;
	};

	struct glGeneratedIndices
	{
		GLuint static_vao, dynamic_vao;
		GLuint shader_program;
		GLuint static_vertex_buffer, dynamic_vertex_buffer;
		GLuint model_to_world_uniform, model_to_world_inverse_transpose_uniform, world_to_clip_uniform, shading_toggle_uniform;
	};

	class App : public Window::Listener
	{
	private:
		enum CurrentModel
		{
			MODEL_EXAMPLE,
			MODEL_USER_GENERATED,
			MODEL_FROM_INDEXED_DATA,
			MODEL_FROM_FILE
		};

	public:
		App();		// constructor
		virtual				~App() {}	// destructor

		virtual bool		handleEvent(const Window::Event& ev);

	private:
		App(const App&);		// forbid copy
		App& operator=(const App&);	// forbid assignment

		void				initRendering();
		void				render();
		std::vector<Vertex>	loadObjFileModel(std::string filename);

		void				streamGeometry(const std::vector<Vertex>& vertices);

		Window				window_;
		CommonControls		common_ctrl_;

		CurrentModel		current_model_;
		bool				model_changed_;
		bool				shading_toggle_;
		bool				shading_mode_changed_;
		bool				camera_trackball_;
		bool				camera_trackball_changed_;
		bool				rotation_animation_;
		bool				rotation_animation_changed;

		bool				simplify_;
		bool				simplify_changed;

		bool				simplify_1000;
		bool				simplify1000_changed;

		glGeneratedIndices	gl_;

		size_t				vertex_count_;

		float				camera_rotation_angle_;

		// YOUR CODE HERE (R1)
		// Add a class member to store the current translation.

		FW::Vec3f translation_;
		float rotation_angle_around_y_;
		float scale_along_x_;
		float movement_velocity_;

#pragma region Trackball Camera	

		VirtualTrackballCamera virtual_trackball_camera_;
		bool left_mouse_button_pressed_;

#pragma endregion 
		float width_;
		float height_;
		float win_FOV_;
		// EXTRA:
		// For animation extra credit you can use the framework's Timer class.
		// The .start() and .unstart() methods start and stop the timer; when it's
		// running, .end() gives you seconds passed after last call to .end().
		Timer timer_;

		void SimplificationAlgorithm();

		QEM_SurfaceSimplification S;
		std::vector<FW::Vec3f> positions;
		std::vector<FW::Vec3f> normals;
		std::vector<std::array<unsigned, 6>>  faces;

		void Simplify(int fNum);

		std::vector<Vertex>	loadPlyFileModel(std::string filename);
		
	};

}