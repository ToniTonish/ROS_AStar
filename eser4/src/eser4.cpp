#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>

#define angle_increment 0.00436736317351
#define SOGLIA 0.15
#define ANGULAR_V 0.5
#define LINEAR_V 0.5
#define ROW 177
#define COLUMN 162

struct node {
	double x;
	double y;
	double g;
	double h;
	struct node* parent;
	struct node* next;
};

typedef struct node* p_node;

bool mappa[ROW][COLUMN];
bool mappa_vertici[ROW][COLUMN];

bool linear_movement = false;
bool angular_movement_left = false;
bool angular_movement_right = false;
bool stop = false;
bool check = true;
bool go_forward = false;
bool ready_astar = false;
bool go = false;
bool one_time = true;
bool control = true;

double robot_pose_x = 0;
double robot_pose_y = 0;
double robot_pose_orientation = 0;

double goal_x;
double goal_y;

//-11.0 18.5

double goal_f_x = 0;
double goal_f_y = 0;

const double k_obstacle = 3.0;
double k_goal = 30.0; //25

double f_tot;

// Funzioni di utilita'
double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

// discretizzazione variabili
int discretize(double var) {
	return (int)round(fabs(3*var));
}

double undiscretize(int var) {
	return ((double) var)/3.0;
}

p_node findNodeAndModify(p_node v, double x, double y, double dist){
	p_node tmp = v;
	p_node temp2;

	//CONTROLLO PRIMO ELEMENTO!!!!!!!!!!!!!!
	if (tmp->x == x && tmp->y == y) {
		if (tmp->g > dist) {
			temp2 = tmp;
			tmp = tmp->next;
			return temp2;
		}
	}
	
	while (tmp->next != NULL) {
		if (tmp->next->x == x && tmp->next->y == y) {
			 temp2 = tmp->next;
			 if(temp2->g > dist){
			 	tmp->next = tmp->next->next;
			 	return temp2;
			 	}
		}
		tmp = tmp->next;
	}
	
	return NULL;
}

// Inserimento ordinato in Node
p_node insertNode(p_node v, double p_x, double p_y, double dist, p_node p) {
	p_node r;
	p_node temp = v ;
	r = (p_node)malloc(sizeof(struct node));
	r->x = p_x;
	r->y = p_y;
	r->g = dist;
	r->h = distance(p_x, p_y, goal_x, goal_y);
	r->next = NULL;
	r->parent = p;
	if (v == NULL || v->g + v->h > r->g + r->h) {
		v = r;
		v->next = temp;
		return r;
	} else {
		while (temp) {
			if (temp->g + temp->h <= r->g + r->h &&
					(temp->next == NULL ||
							(temp->next->g + temp->next->h > r->g + r->h)))
			{
				r->next = temp->next;
				temp->next=r;
				return v;
			}
			temp = temp->next;
		}
	}
	return temp;
}

void printGraph(p_node vertex) {
	p_node tmp = vertex;
	while (tmp != NULL) {
		printf("distance: %f\n", tmp->g);
		printf("heuristic distance: %f\n", tmp->h);
		printf("point x: %f\n", tmp->x);
		printf("point y: %f\n", tmp->y);
		tmp = tmp->next;
	}
}

// ****************************************************************************
p_node insertInHead(p_node r, p_node v) {
	if (r == NULL) {
		r = v;
		r->next = NULL;
	} else {

		p_node temp = r;
		r = v;
		r->next=temp;
		/*v->next = r;
		r = v;*/
	}
	return r;
}
// **********************************************************

// Cancellazione del primo elemento della lista di archi
p_node deleteHeadVertex(p_node* vertex) {
	p_node s = NULL;

	if(vertex == NULL) {
		printf("eliminazione testa da lista vuota\n");
		exit(1);
	}
	s = *vertex;
	*vertex = (*vertex)->next;
	s->next = NULL;

	return s;
}

// Ricerca un nodo dentro una lista 
bool findNode(p_node v, double x, double y){
	p_node tmp = v;
	while (tmp != NULL) {
		if (tmp->x == x && tmp->y == y) {
			return true;
		}
		tmp = tmp->next;
	}
	return false;
}

// callback di pose per prendere le posizioni del robot
void subPose(const nav_msgs::Odometry::ConstPtr& pos)
{
	// setto le posizioni iniziali del robot 
	robot_pose_x = pos->pose.pose.position.x;
	robot_pose_y = pos->pose.pose.position.y;
	robot_pose_orientation = tf::getYaw(pos->pose.pose.orientation);	

	//printf("posx posy posw: [%f] [%f] [%f]\n", robot_pose_x, robot_pose_y, robot_pose_orientation*180/M_PI);
	
	if(check)
		check = false; 
}

void salvaMappa(const char *destinazione, bool b){
	FILE *fp;
	fp = fopen(destinazione, "wb");
	if (fp == NULL) {
	printf("errore apertura file.\n");
		return ;
	}
	fputs("P5 177 162 255 ", fp);
	for (int j=COLUMN-1;j>=0;j--){
		for (int i=ROW-1;i>=0;i--){
			unsigned char temp;
			if (b) {
				temp = mappa_vertici[i][j] ? 0 : 255; 		
			} else {
				temp = mappa[i][j] ? 0 : 255; 
			}
			fwrite(&temp, sizeof(unsigned char), 1, fp);
		}
	}
	fclose(fp);
}

void letturaMappa(const char *destinazione) {
	FILE *fp;
	fp = fopen(destinazione, "r");
	if (fp == NULL) {
		printf("errore apertura file.\n");
		return ;
	}
	fscanf(fp, "P5 177 162 255 ");
	
	for (int j = COLUMN-1; j >= 0; j--){
		for (int i=ROW-1;i>=0;i--){
			unsigned char temp;
			
			fscanf(fp, "%c", &temp);
			if (temp == 0) {
				 mappa[i][j] = true;
			} else {
				 mappa[i][j] = false;
			}
		}
	}
}
void aggiornaMappaVertici() {
	for (int i=0;i<ROW-1;i++) {
		for (int j=0;j<COLUMN-1;j++) {
			bool temp[2][2] = {false}; 	
			temp[0][0] = mappa[i][j];
			temp[0][1] = mappa[i][j+1];
			temp[1][0] = mappa[i+1][j];
			temp[1][1] = mappa[i+1][j+1];
			if (temp[0][0] == true && temp[0][1] == false && temp[1][0] == false && temp[1][1] == false) {
				mappa_vertici[i+1][j+1] = true;
			} else if (temp[0][0] == false && temp[0][1] == true && temp[1][0] == false && temp[1][1] == false) {
				mappa_vertici[i+1][j] = true;
			} else if (temp[0][0] == false && temp[0][1] == false && temp[1][0] == true && temp[1][1] == false) {
				mappa_vertici[i][j+1] = true;
			} else if (temp[0][0] == false && temp[0][1] == false && temp[1][0] == false && temp[1][1] == true) {
				mappa_vertici[i][j] = true;
			}	
		}
	}
}

void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan) {
	double f_repx, f_repy;
	double f_attrx, f_attry;
	double angle_from_goal, distance_from_goal, distance_from_goal_fake;
	double angle_tot;

	// questo mi serve per rilevare gli ostacoli e calcolare la forza repulsiva data da essi
	// e fare la somma per calcolarmi le componenti repulsive totali.
	for(int i=0;i<1080;i++) {
			f_repx += cos((i*angle_increment)-(3*M_PI/4))/(k_obstacle*(pow(scan->ranges[i], 2))); 
			f_repy += sin((i*angle_increment)-(3*M_PI/4))/(k_obstacle*(pow(scan->ranges[i], 2))); 
	}

	//calcolo la distanza del robot dal goal
	distance_from_goal = sqrt(pow(goal_x-robot_pose_x, 2)+(pow(goal_y-robot_pose_y, 2)));
	
	distance_from_goal_fake = sqrt(pow(goal_f_x-robot_pose_x, 2)+(pow(goal_f_y-robot_pose_y, 2)));
	
	if (distance_from_goal_fake < 1)
		k_goal = 100;
	if (distance_from_goal_fake < 2 && distance_from_goal_fake > 1)
		k_goal = 85;
	if (distance_from_goal_fake < 3 && distance_from_goal_fake > 2)
		k_goal = 70;
	if (distance_from_goal_fake < 4 && distance_from_goal_fake > 3)
		k_goal = 50;
	if (distance_from_goal_fake > 4)
		k_goal = 30;

	//calcolo l'angolo tra la posizione frontale del robot e il goal
	if (control) {
		angle_from_goal = atan2(goal_y-robot_pose_y, goal_x-robot_pose_x) - robot_pose_orientation;
	} else {
		angle_from_goal = atan2(goal_f_y-robot_pose_y, goal_f_x-robot_pose_x) - robot_pose_orientation;
	}
	
	if(angle_from_goal < -M_PI)
		angle_from_goal += 2*M_PI;
	else if (angle_from_goal > M_PI)
		angle_from_goal -= 2*M_PI;

	f_attrx = cos(angle_from_goal)*k_goal*distance_from_goal;
	f_attry = sin(angle_from_goal)*k_goal*distance_from_goal;

	angle_tot = atan2(f_attry-f_repy, f_attrx-f_repx);
	if (angle_tot < -M_PI)
		angle_tot += 2*M_PI;
	else if (angle_tot > M_PI)
		angle_tot -= 2*M_PI;
		
	f_tot = sqrt(pow(f_attrx-f_repx, 2)+pow(f_attry-f_repy, 2));
	angle_tot = atan2(f_attry-f_repy, f_attrx-f_repx);

	if (stop == false) {
		if (fabs(angle_tot) > SOGLIA) {
			if (angle_tot > 0) {
				angular_movement_left = true;
				angular_movement_right = false;
			}
			else {
				angular_movement_right = true;
				angular_movement_left = false;
			}
		} else if (go){
			angular_movement_right = false;
			angular_movement_left = false;
			linear_movement = true;
		}
		else {
			stop = true;
			go_forward = true;
		}
	} 
}

bool raggiungibile(int x1, int y1, int x2, int y2) {
	
	int temp_x1;
	int temp_x2;
	int temp_y1;
	int temp_y2;

	if (x1 > x2) {
			temp_x1 = x1;
			temp_x2 = x2;
			temp_y1 = y1;
			temp_y2 = y2;

		} else {
			temp_x1 = x2;
			temp_x2 = x1;
			temp_y1 = y2;
			temp_y2 = y1;
		}

	while (1) {
		if (temp_x1 == temp_x2) {
			if (temp_y1 > temp_y2) {
					if (mappa[temp_x1][temp_y1-1] || mappa[temp_x2][temp_y2+1]) {
						return false;
					} else if((temp_y1-temp_y2) <= 2){ 
						return true;
					} else {
						temp_y1--;
						temp_y2++;
						}
			} else {
				if (mappa[temp_x1][temp_y1+1] || mappa[temp_x2][temp_y2-1]) {
					return false;
				} else if ((temp_y2-temp_y1) <= 2){ 
					return true;
				} else {							
					temp_y1++;
					temp_y2--;
				}
			}
		} else if (temp_y1 == temp_y2) {

					if (mappa[temp_x1-1][temp_y1] || mappa[temp_x2+1][temp_y2]) {
						return false;
					}else if ((temp_x1-temp_x2) <= 2){ 
						return true;

					} else {
						temp_x1--;
						temp_x2++;
					}

		} else {
				if (temp_y1 > temp_y2 ){
						if (mappa[temp_x1][temp_y1-1] || mappa[temp_x1-1][temp_y1] || mappa[temp_x1-1][temp_y1-1]) {
									return false;
								}
								if (mappa[temp_x2][temp_y2+1] || mappa[temp_x2+1][temp_y2] || mappa[temp_x2+1][temp_y2+1]) {
									return false;
								}
								if ((temp_x1-temp_x2  <= 2) && (temp_y1-temp_y2 <= 2)) {
									return true;
								} else if (temp_x1-temp_x2<= 2) {
									temp_y1--;
									temp_y2++;
								} else if (temp_y1-temp_y2 <= 2) {
									temp_x1--;
									temp_x2++;
								} else {
									temp_x1--;
									temp_y1--;
									temp_x2++;
									temp_y2++;
								}
						} else {
								if (mappa[temp_x1][temp_y1+1] || mappa[temp_x1-1][temp_y1] || mappa[temp_x1-1][temp_y1+1]) {
												return false;
											}
								if (mappa[temp_x2][temp_y2-1] || mappa[temp_x2+1][temp_y2] || mappa[temp_x2+1][temp_y2-1]) {
												return false;
											}
								if ((temp_x1-temp_x2  <= 2) && (temp_y2-temp_y1 <= 2)) {
												return true;
											} else if (temp_x1-temp_x2<= 2) {
												temp_y1++;
												temp_y2--;
											} else if (temp_y2-temp_y1 <= 2) {
												temp_x1--;
												temp_x2++;
											} else {
												temp_x1--;
												temp_y1++;
												temp_x2++;
												temp_y2--;
											}
								}
					}
			}
	return false;
}

// AStar per trovare cammino minimo
p_node AStar() {
	ready_astar = false;
	p_node CLOSE = NULL;
	p_node OPEN = NULL;
	OPEN = insertNode(OPEN, (int)round(fabs(3*robot_pose_x)), (int)round(fabs(3*robot_pose_y)), 0, NULL);
	

	while (OPEN != NULL) {

		p_node tmp = deleteHeadVertex(&OPEN);

		if(raggiungibile(discretize(goal_x), discretize(goal_y), tmp->x, tmp->y)){
			return tmp; 
		}

		CLOSE = insertNode(CLOSE, tmp->x, tmp->y, tmp->g, tmp->parent);
		
		for (int i=0; i<ROW-1;i++){
			for (int j=0;j<COLUMN-1;j++){
				if (mappa_vertici[i][j]){
					if (raggiungibile(i, j, tmp->x, tmp->y)){
						if (findNode(CLOSE, i, j)){
						} else if (findNode(OPEN, i, j)) {
							p_node t = findNodeAndModify(OPEN, i, j, tmp->g + distance(tmp->x, tmp->y, i, j));
							if (t != NULL){
								t->g = tmp->g +  distance(tmp->x, tmp->y, i, j);
								OPEN = insertNode(OPEN, t->x, t->y, t->g, tmp);
							}
							//t->g =  distance(tmp->x, tmp->y, i, j);
							//OPEN = insertNode(OPEN, t->x, t->y, t->g, tmp);
						} else {
							if(tmp->parent){
								double temp_distance = tmp->g + distance(tmp->x, tmp->y, i, j);
								OPEN = insertNode(OPEN, i, j, temp_distance, tmp);
							} else {
								double temp_distance = distance((int)round(fabs(3*robot_pose_x)), (int)round(fabs(3*robot_pose_y)), i, j);
								OPEN = insertNode(OPEN, i, j, temp_distance, tmp);
							}
						}
					}
				}
			}
		}
	}
	//ritorna fallimento
	return NULL;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eser4");
	
	ros::NodeHandle n; 
	ros::Subscriber read_position, read_laser;
	
	goal_x = atof(argv[1]);
 	goal_y = atof(argv[2]);
	
	letturaMappa("/home/tonish/Desktop/mappa.pgm");
	
	aggiornaMappaVertici();
	salvaMappa("/home/tonish/Desktop/vertici.pgm", true);
	
	read_position = n.subscribe("/base_pose_ground_truth", 10, subPose);
	read_laser = n.subscribe("/base_scan", 10, laserCallBack);
	
	ros::Publisher twist_pub;
	geometry_msgs::Twist msg;
	twist_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	p_node path = NULL;
	p_node best_node = NULL;
	p_node final_goal = NULL;
	final_goal = insertNode(final_goal, (int)round(fabs(3*goal_x)), (int)round(fabs(3*goal_y)), 0, NULL);
	
	ros::Rate loop_rate(10);
	
	while(check)
		ros::spinOnce();
	
	while(ros::ok()) {
	
		if (fabs(robot_pose_x-goal_x) < 0.5 && fabs(robot_pose_y-goal_y) < 0.5) {
			printf("GOAL RAGGIUNTO!!\n");
			ros::shutdown();
		}
	
		if (robot_pose_orientation > M_PI)
			robot_pose_orientation -= 2*M_PI;
		
		
		if (f_tot < 100) {		
			msg.linear.x = LINEAR_V;
			msg.angular.z = 0.0;
		} else {
			
			if (linear_movement) {		
				msg.linear.x = LINEAR_V; 
				msg.angular.z = 0.0;
			}	
			
			if (angular_movement_left) {
				msg.linear.x = 0.0;
				msg.angular.z = ANGULAR_V;
			}
		
			if (angular_movement_right) {
				msg.linear.x = 0.0;
				msg.angular.z = -ANGULAR_V;
			}	
		}
							
		if (stop && one_time) { 
			msg.linear.x = 0.0;
			msg.angular.z = 0.0;
			one_time = false;
		}	
		
		if (stop) {
			p_node temp = AStar();
			path = insertInHead(path, final_goal);
			
			while (temp->parent) {
				path = insertInHead(path, temp);
				temp = temp->parent;
			}
			
			goal_f_x = undiscretize(-path->x);
			goal_f_y = undiscretize(path->y);

			printf("coordinate del primo vertice= x:%f, y:%f \n", goal_f_x, goal_f_y);
			
			stop = false;
			go = true;
			control = false;
		}
		
		if (fabs(robot_pose_x-goal_f_x) < 0.1 && fabs(robot_pose_y-goal_f_y) < 0.1) {
			if (path->next) {
				path = path->next;
				goal_f_x = undiscretize(-path->x);
				goal_f_y = undiscretize(path->y);
				
				printf("coordinate del vertice corrente= x:%f, y:%f \n", goal_f_x, goal_f_y);
			}
		} 
		
		twist_pub.publish(msg);
		
		loop_rate.sleep();

		ros::spinOnce();
	
	}
}
