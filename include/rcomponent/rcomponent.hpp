#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/utils/factory.hpp"
#include "rcomponent/utils/log_macros.hpp"
#include "rcomponent/state/state_manager.hpp"

// Comprobar clock si use_sim es true
// Leer de freq desde params OK
// Revisar comentarios headers y dependencias
// Añadir minima doc para doxygen OK
// Añadir readme 
// Revisar desde laser_scan formas declarar nodo y suscribirse
// Añadir test minimo

// Puntos a comentar
// Formato logs: ¿mostrar nombre del archivo o funcion? 
// Revisar mensajes Status y NodeStatus OK
// Maquina de estados OK
//  - Nombres para usar ready -> running
//  - Usamos directamente los de lifecycle
//  - La maquina de estados es lifecycle, los estados operacionales del nodo son reactivos

// Añadir healthcheck
// Llamar a funcion on_configure -> configure a rcomponent, devoolver lo mismo

namespace rcomponent
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Rcomponent : public rclcpp_lifecycle::LifecycleNode
{

	public:

		Rcomponent() = delete;
		explicit Rcomponent(const std::string& node_name);
	
		void init();

		rclcpp::Logger logger_;

	  // Lifecycle callbacks
		CallbackReturn on_configure(const rclcpp_lifecycle::State &);

		CallbackReturn on_activate(const rclcpp_lifecycle::State &);

		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

		CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

		CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

		CallbackReturn on_error(const rclcpp_lifecycle::State &);


		// Control loop
		virtual void control_loop();

		rclcpp::TimerBase::SharedPtr timer_;

	private:

		std::shared_ptr<StateManager> rmanager_;

		double frequency_{1.0};
};

}