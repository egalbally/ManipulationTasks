/**
 * optitrack_main.cpp
 *
 * Publish OptiTrack data to Redis from recorded CSV data or over the network.
 *
 * Author: Toki Migimatsu
 * Created: July 2017
 */

#include "optitrack/OptiTrack.h"
#include "optitrack/OptiTrackClient.h"
#include "redis/RedisClient.h"

#include <string>
#include <vector>

#include <Eigen/Core>

using namespace OptiTrack;

int main(int argc, char** argv) {

	Eigen::Matrix3f R_opti_to_kuka;
	R_opti_to_kuka <<  0, 0, -1, 
	                  -1, 0, 0, 
	                   0, 1, 0;
	const Eigen::Quaternionf quat_opti_to_kuka(R_opti_to_kuka);

	// Usage
	if (argc < 3) {
		std::cout << "Usage: optitrack_redis [-i INPUT_CSV_FILE]" << std::endl
		          << "                       [-l LOCAL_PUBLIC_IP] [-s OPTITRACK_SERVER_IP]" << std::endl
		          << "                       [-rs REDIS_SERVER_IP] [-rp REDIS_SERVER_PORT]" << std::endl
		          << std::endl
		          << "Publish OptiTrack data to Redis from recorded CSV data or live over the network." << std::endl
		          << std::endl
		          << "Optional arguments:" << std::endl
		          << "  -i INPUT_CSV_FILE" << std::endl
		          << "\t\t\t\tInput csv file recorded with Motive." << std::endl
		          << "  -l LOCAL_PUBLIC_IP" << std::endl
		          << "\t\t\t\tLocal public IP (required if connecting over the network)." << std::endl
		          << "  -s OPTITRACK_SERVER_IP" << std::endl
		          << "\t\t\t\tOptiTrack server IP (default " << OptiTrackServer::DEFAULT_IP << ")." << std::endl
		          << "  -rs REDIS_SERVER_IP" << std::endl
		          << "\t\t\t\tRedis server IP (default " << RedisServer::DEFAULT_IP << ")." << std::endl
		          << "  -rp REDIS_SERVER_PORT" << std::endl
		          << "\t\t\t\tRedis server port (default " << RedisServer::DEFAULT_PORT << ")." << std::endl;
		exit(0);
	}

	// Parse arguments
	std::string optitrack_ip = OptiTrackServer::DEFAULT_IP;
	std::string local_ip;
	std::string csv_file;
	std::string redis_ip = RedisServer::DEFAULT_IP;
	int redis_port = RedisServer::DEFAULT_PORT;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-s")) {
			// Optitrack server IP
			optitrack_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-l")) {
			// Local public IP
			local_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-i")) {
			// Input CSV file
			csv_file = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-rs")) {
			// Redis server IP
			redis_ip = std::string(argv[++i]);
		} else if (!strcmp(argv[i], "-rp")) {
			// Redis server port
			sscanf(argv[++i], "%d", &redis_port);
		}
	}

	// Connect to Redis
	RedisClient redis;
	redis.connect(redis_ip, redis_port);
	std::cout << "Connected to Redis server: " << redis_ip << ":" << redis_port << std::endl;

	// Connect to OptiTrack
	OptiTrackClient optitrack;
	if (!csv_file.empty()) {
		std::cout << "Opening CSV file: " << csv_file << std::endl;
		optitrack.openCsv(csv_file);
	} else if (!local_ip.empty()) {
		std::cout << "Opening network connection: " << local_ip << " to " << optitrack_ip << std::endl;
		optitrack.openConnection(local_ip, optitrack_ip);
	} else {
		std::cout << "Need to specify a CSV file or the local public IP address." << std::endl;
		exit(1);
	}

	// Read frames from OptiTrackClient
	std::cout << "Publishing data to Redis..." << std::endl;
	while (true) {
		if (!optitrack.getFrame()) continue;

		double t_curr = optitrack.frameTime();
		Eigen::MatrixXf pos_rigid_bodies(optitrack.pos_rigid_bodies_.size(), 3);
		Eigen::MatrixXf ori_rigid_bodies(optitrack.pos_rigid_bodies_.size(), 4);
		Eigen::MatrixXf pos_single_markers(optitrack.pos_single_markers_.size(), 3);
		for (int i = 0; i < optitrack.pos_rigid_bodies_.size(); i++) {
			pos_rigid_bodies.row(i) = R_opti_to_kuka * optitrack.pos_rigid_bodies_[i];
		}
		for (int i = 0; i < optitrack.ori_rigid_bodies_.size(); i++) {
			Eigen::Quaternionf ori = quat_opti_to_kuka * optitrack.ori_rigid_bodies_[i] * quat_opti_to_kuka.conjugate();
			ori_rigid_bodies.row(i) = ori.coeffs();
		}
		for (int i = 0; i < optitrack.pos_single_markers_.size(); i++) {
			pos_single_markers.row(i) = optitrack.pos_single_markers_[i];
		}

		redis.pipeset({
			{KEY_TIMESTAMP, std::to_string(t_curr)},
			{KEY_POS_RIGID_BODIES, RedisClient::encodeEigenMatrix(pos_rigid_bodies)},
			{KEY_ORI_RIGID_BODIES, RedisClient::encodeEigenMatrix(ori_rigid_bodies)},
			{KEY_POS_SINGLE_MARKERS, RedisClient::encodeEigenMatrix(pos_single_markers)},
		});
	}

	return 0;
}
