#include <WiFi.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// Definir les credencials de la xarxa Wi-Fi
const char* ssid = "NomDeLaTevaXarxaWiFi";
const char* password = "LaTevaContrasenya";

// Definir el node, publicador i subscriptor de ROS 2
rcl_node_t node;
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
rclc_support_t support;
rclc_client_t client;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Missatges ROS 2
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// Variables per a la lectura d'encoders i control de motors (a adaptar)
long left_encoder_count = 0;
long right_encoder_count = 0;
float linear_velocity = 0.0;
float angular_velocity = 0.0;

// Funcions per a la lectura d'encoders i control de motors (HAURIES D'ADAPTAR-LES)
long readEncoder(int wheel) {
  // ... la teva implementació per llegir l'encoder ...
  return 0; // Placeholder
}

void setMotorSpeeds(int left_speed, int right_speed) {
  // ... la teva implementació per controlar els motors ...
}

// Funció de callback per al tòpic /cmd_vel
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_velocity = msg->linear.x;
  angular_velocity = msg->angular.z;
  // Aquí hauries de convertir les velocitats lineal i angular a velocitats de les rodes
  // i cridar la funció setMotorSpeeds()
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnectat a WiFi");

  // Inicialitzar l'al·locador
  allocator = rcl_get_default_allocator();

  // Inicialitzar el suport de ROS 2
  rclc_support_init(&support, 0, NULL, &allocator);

  // Crear el node ROS 2
  rclc_node_init_default(&node, "arduino_nano_node", "", &support);

  // Crear el publicador per a /odom
  const rmw_qos_profile_t odom_qos = rmw_qos_profile_default;
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom");

  // Crear el subscriptor per a /cmd_vel
  const rmw_qos_profile_t cmd_vel_qos = rmw_qos_profile_default;
  rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  // Inicialitzar el missatge d'odometria (omple els camps necessaris)
  odom_msg.header.frame_id.data = (char *) "odom";
  odom_msg.child_frame_id.data = (char *) "base_link";
  odom_msg.pose.covariance[0] = 0.01; // Exemple de covariància

  // Inicialitzar l'executor
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 1, &allocator); // Un únic subscriptor

  // Afegir el subscriptor a l'executor
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, &allocator);
}

void loop() {
  // Llegir els encoders
  left_encoder_count = readEncoder(LEFT);
  right_encoder_count = readEncoder(RIGHT);

  // Calcular l'odometria (HAURIES D'IMPLEMENTAR AQUESTA LÒGICA)
  // Omplir el missatge odom_msg amb les dades calculades
  odom_msg.pose.pose.position.x += ...;
  odom_msg.pose.pose.orientation.z = ...;
  odom_msg.twist.twist.linear.x = ...;
  odom_msg.twist.twist.angular.z = ...;
  odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
  odom_msg.header.stamp.sec = rmw_uros_epoch_seconds();

  // Publicar el missatge d'odometria
  rcl_publish(&odom_publisher, &odom_msg, NULL);

  // Fer girar l'executor per processar els callbacks (p. ex., /cmd_vel)
  rclc_executor_spin_some(&executor, 100);

  delay(10);
}