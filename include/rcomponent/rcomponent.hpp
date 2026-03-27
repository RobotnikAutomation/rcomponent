#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rcomponent/publisher.hpp"
#include "rcomponent/subscriptor.hpp"
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
// Comunicar con StateManager para que sepa que el subscriptor está activo o no
// Revisar multithread por defecto OK
// Gestionar returns de los on_configure... OK
// Revisar pub/sub para planitlla OK
// Probar y cerrar todo

// Llamar a funcion on_configure -> configure a rcomponent, devoolver lo mismo

// Añadir colores a los logs por terminal

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

	protected:

		template<typename MsgT>
		std::shared_ptr<ManagedPublisher<MsgT>> create_rc_publisher(
			const std::string& topic
		)
		{
				auto pub = std::make_shared<ManagedPublisher<MsgT>>(this, topic);
				pub->topic_name = topic;
				registered_rc_publishers_.push_back(pub);
				return pub;
		}

		template<typename MsgT>
		std::shared_ptr<ManagedSubscriptor<MsgT>> create_rc_subscription(
			const std::string& topic,
			std::function<void(typename MsgT::SharedPtr)> user_callback
		)
		{
				auto sub = std::make_shared<ManagedSubscriptor<MsgT>>(this, topic, user_callback);
				sub->topic_name = topic;
				registered_rc_subscriptors_.push_back(sub);
				return sub;
		}

		virtual CallbackReturn rc_configure() = 0;
    virtual CallbackReturn rc_activate() = 0;
		virtual CallbackReturn rc_dectivate() = 0;
		virtual CallbackReturn rc_cleanup() = 0;
		virtual CallbackReturn rc_shutdown() = 0;
		virtual CallbackReturn rc_error() = 0;

		virtual void rc_loop() = 0;

	private:

		double loop_frequency_{1.0};

		rclcpp::TimerBase::SharedPtr timer_;
		std::shared_ptr<StateManager> rmanager_;

	  // Lifecycle callbacks
		CallbackReturn on_configure(const rclcpp_lifecycle::State &);
		CallbackReturn on_activate(const rclcpp_lifecycle::State &);
		CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
		CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
		CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
		CallbackReturn on_error(const rclcpp_lifecycle::State &);
		
		std::vector<std::shared_ptr<ManagedPublisherInterface>> registered_rc_publishers_;
		std::vector<std::shared_ptr<ManagedSubscriptorInterface>> registered_rc_subscriptors_;

};
}