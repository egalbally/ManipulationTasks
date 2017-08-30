/**
 * OptiTrackClient.cpp
 *
 * Author: Toki Migimatsu
 * Created: April 2017
 */

#include "OptiTrackClient.h"

// NatNetLinux
#include <NatNetLinux/NatNet.h>

// std
#include <iostream>
#include <math.h>

OptiTrackClient::~OptiTrackClient() {
	closeConnection();
	closeCsv();
}

bool OptiTrackClient::openConnection(const std::string& local_public_ip_address, const std::string& server_ip_address) {
    uint32_t local_address = inet_addr(local_public_ip_address.c_str());
    uint32_t server_address = inet_addr(server_ip_address.c_str());

	// Use this socket address to send commands to the server.
	struct sockaddr_in server_commands = NatNet::createAddress(server_address, NatNet::commandPort);

	// Create sockets
	sd_command_ = NatNet::createCommandSocket(local_address);
	sd_data_ = NatNet::createDataSocket(local_address);

	// Start the CommandListener in a new thread.
	command_listener_ = std::make_unique<CommandListener>(sd_command_);
	command_listener_->start();

	// Send a ping packet to the server so that it sends us the NatNet version
	// in its response to commandListener.
	NatNetPacket ping = NatNetPacket::pingPacket();
	ssize_t status = ping.send(sd_command_, server_commands);

	// Wait here for ping response to give us the NatNet version.
	unsigned char natnet_major;
	unsigned char natnet_minor;
	bool success = command_listener_->getNatNetVersion(natnet_major, natnet_minor);
	if (!success) {
		std::cout << "OptiTrackClient: Failed to connect to OptiTrack server " << server_ip_address << "." << std::endl << std::endl;
		command_listener_->stop();
		command_listener_->join();
		command_listener_.release();
		close(sd_command_);
		close(sd_data_);
		return false;
	}

	// Start up a FrameListener in a new thread.
	frame_listener_ = std::make_unique<FrameListener>(sd_data_, natnet_major, natnet_minor);
	frame_listener_->start();
	return true;
}

void OptiTrackClient::closeConnection() {
	// Wait for threads to finish
	if (command_listener_) command_listener_->stop();
	if (frame_listener_) frame_listener_->stop();
	if (command_listener_) command_listener_->join();
	if (frame_listener_) frame_listener_->join();

	// Close sockets
	close(sd_command_);
	close(sd_data_);

	// Reset state for next connection
	resetState();
}

bool OptiTrackClient::openCsv(const std::string& filename) {
	csv_file_.open(filename);
	if (csv_file_.fail()) return false;

	int num_rigid_bodies = 0;
	int num_markers = 0;

	std::string line, cell;
	std::stringstream ss_line;
	std::getline(csv_file_, line);  // First line
	std::getline(csv_file_, line);  // Empty line

	std::getline(csv_file_, line);  // Data type
	ss_line.str(line);
	std::getline(ss_line, cell, ',');  // Frame
	std::getline(ss_line, cell, ',');  // Time
	marker_types_.clear();
	while (std::getline(ss_line, cell, ',')) {
		if (cell == "Rigid Body") {
			marker_types_.push_back(RIGID_BODY_POSITION);
			marker_types_.push_back(RIGID_BODY_POSITION);
			// Skip -YZWXYZe
			for (int i = 0; i < 7; i++) std::getline(ss_line, cell, ',');
			num_rigid_bodies++;
		} else if (cell == "Marker") {
			marker_types_.push_back(SINGLE_MARKER_POSITION);
			// Skip -YZ
			for (int i = 0; i < 2; i++) std::getline(ss_line, cell, ',');
			num_markers++;
		} else {
			marker_types_.push_back(OTHER);
		}
	}

	std::getline(csv_file_, line);  // Label
	ss_line.str("");
	ss_line.clear();
	ss_line.str(line);
	std::getline(ss_line, cell, ',');  // Frame
	std::getline(ss_line, cell, ',');  // Time
	int idx = 0;
	while (std::getline(ss_line, cell, ',')) {
		if (marker_types_[idx] == RIGID_BODY_POSITION) {
			// Skip -YZWXYZe
			for (int i = 0; i < 7; i++) std::getline(ss_line, cell, ',');
			idx++;
		} else if (marker_types_[idx] == SINGLE_MARKER_POSITION) {
			if (cell.compare(0, 7, "Marker_") != 0) {
				// Remove rigid body marker from single markers list
				marker_types_[idx] = OTHER;
				marker_types_.insert(marker_types_.begin() + idx, 2, OTHER);
				idx += 2;
			}
			// Skip -YZ
			for (int i = 0; i < 2; i++) std::getline(ss_line, cell, ',');
		}
		idx++;
	}


	std::getline(csv_file_, line);  // Marker ID

	std::getline(csv_file_, line);  // Position/Orientation
	ss_line.str("");
	ss_line.clear();
	ss_line.str(line);
	std::getline(ss_line, cell, ',');  // Frame
	std::getline(ss_line, cell, ',');  // Time
	idx = 0;
	while (std::getline(ss_line, cell, ',')) {
		if (cell == "Rotation") {
			marker_types_[idx] = RIGID_BODY_ORIENTATION;
			// Skip -YZW
			for (int i = 0; i < 3; i++) std::getline(ss_line, cell, ',');
		} else if (marker_types_[idx] == RIGID_BODY_POSITION) {
			// Skip -YZe
			for (int i = 0; i < 3; i++) std::getline(ss_line, cell, ',');
		} else if (marker_types_[idx] == SINGLE_MARKER_POSITION) {
			// Skip -YZ
			for (int i = 0; i < 2; i++) std::getline(ss_line, cell, ',');
		}
		idx++;
	}

	std::getline(csv_file_, line);  // XYZW

	// Set buffer sizes
	pos_rigid_bodies_.resize(num_rigid_bodies);
	for (auto& pos_rigid_body : pos_rigid_bodies_) pos_rigid_body.fill(NAN);

	ori_rigid_bodies_.resize(num_rigid_bodies);
	for (auto& ori_rigid_body : ori_rigid_bodies_) ori_rigid_body.coeffs().fill(NAN);

	num_single_marker_ids_ = num_markers;
	pos_single_markers_.resize(num_markers);
	for (auto& pos_single_marker : pos_single_markers_) pos_single_marker.fill(NAN);

	return true;
}

void OptiTrackClient::closeCsv() {
	csv_file_.close();
	resetState();
}

bool OptiTrackClient::setFps(int fps) {
	if (csv_file_.is_open() || command_listener_ || frame_listener_) {
		std::cerr << "OptiTrackClient::setFps() : Cannot set fps while connection is open." << std::endl;
		return false;
	}
	fps_ = fps;
	return true;
}

bool OptiTrackClient::getFrame() {
	if (csv_file_.is_open()) return readCsvFrame();
	if (command_listener_ && frame_listener_) return readNetworkFrame();
	// std::err << "OptiTrackClient::getFrame() : No open connection or specified csv file." << std::endl;
	return false;
}

bool OptiTrackClient::readNetworkFrame() {
	// Try to get a new frame from the listener.
	bool frame_available;
	MocapFrame frame(frame_listener_->pop(&frame_available).first);
	if (!frame_available) return false;

	// Get recorded time from frame number
	int t_frame_curr = frame.frameNum();
	if (!t_initialized_) {
		t_frame_start_ = t_frame_curr;
		t_initialized_ = true;
	}
	t_frame_ = t_frame_curr - t_frame_start_;

	// Get rigid bodies
	std::vector<RigidBody> rigid_bodies = frame.rigidBodies();
	pos_rigid_bodies_.resize(rigid_bodies.size());
	ori_rigid_bodies_.resize(rigid_bodies.size());

	for (size_t i = 0; i < rigid_bodies.size(); i++) {
		Point3f pos = rigid_bodies[i].location();
		Quaternion4f ori = rigid_bodies[i].orientation();
		pos_rigid_bodies_[i] = Eigen::Vector3f(pos.x, pos.y, pos.z);
		ori_rigid_bodies_[i] = Eigen::Quaternionf(ori.qw, ori.qx, ori.qy, ori.qz);
	}

	// Get unidentified markers
	std::vector<Point3f> single_markers = frame.unIdMarkers();
	pos_single_markers_.resize(single_markers.size());

	for (size_t i = 0; i < pos_single_markers_.size(); i++) {
		Point3f pos = single_markers[i];
		pos_single_markers_[i] = Eigen::Vector3f(pos.x, pos.y, pos.z);
	}

	return true;
}

bool OptiTrackClient::readCsvFrame() {
	if (csv_file_.eof()) return false;

	// Check time
	if (!t_initialized_) {
		// Initialize time and continue
		t_start_ = std::chrono::high_resolution_clock::now();
		t_initialized_ = true;
	} else {
		// Calculate current frame time
		auto t_curr = std::chrono::high_resolution_clock::now();
		size_t t_frame_curr = fps_ * std::chrono::duration_cast<std::chrono::duration<double>>(t_curr - t_start_).count();
		int num_frames_diff = t_frame_curr - t_frame_next_;

		// Return if not yet time for next frame
		if (num_frames_diff < 0) return false;

		// Set next frame time
		t_frame_ = t_frame_next_;
		t_frame_next_ += 1 + num_frames_diff;

		// Fast forward to most recent frame
		while (num_frames_diff > 0) {
			std::string line;
			std::getline(csv_file_, line);
			num_frames_diff--;
		}
	}

	// Read line
	std::string line;
	if (!std::getline(csv_file_, line)) return false;
	std::stringstream ss_line(line);

	// Parse frame number and time in seconds
	int num_frame;
	double sec_frame;
	char comma;
	ss_line >> num_frame >> comma >> sec_frame >> comma;

	// Parse frame
	int idx_pos_rigid_body = 0;
	int idx_ori_rigid_body = 0;
	int idx_pos_single_marker = 0;
	// pos_single_markers_.resize(num_single_marker_ids_);
	for (auto marker_type : marker_types_) {
		if (marker_type == RIGID_BODY_POSITION) {
			// Parse rigid body position: x,y,z,err
			float x = NAN, y = NAN, z = NAN, e = NAN;
			ss_line >> x >> comma >> y >> comma >> z >> comma >> e;
			if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
				pos_rigid_bodies_[idx_pos_rigid_body] = Eigen::Vector3f(x, y, z);
			idx_pos_rigid_body++;
		} else if (marker_type == RIGID_BODY_ORIENTATION) {
			// Parse rigid body orientation: x,y,z,w
			float w = NAN, x = NAN, y = NAN, z = NAN;
			ss_line >> x >> comma >> y >> comma >> z >> comma >> w;
			if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
				ori_rigid_bodies_[idx_ori_rigid_body] = Eigen::Quaternionf(w, x, y, z);
			idx_ori_rigid_body++;
		} else if (marker_type == SINGLE_MARKER_POSITION) {
			if (ss_line.peek() == ',') {
				// Skip missing marker positions
				for (int i = 0; i < 2; i++) ss_line >> comma;
			} else {
				// Parse single marker position: x,y,z
				float x = NAN, y = NAN, z = NAN;
				ss_line >> x >> comma >> y >> comma >> z;
				if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
					pos_single_markers_[idx_pos_single_marker] = Eigen::Vector3f(x, y, z);
				idx_pos_single_marker++;
			}
		} else {
			// Skip cell
			float f;
			ss_line >> f;
		}
		// Parse comma
		ss_line >> comma;
	}
	// pos_single_markers_.resize(idx_pos_single_marker);

	return true;
}

void OptiTrackClient::resetState() {
	t_initialized_ = false;
	t_frame_ = 0;
	t_frame_next_ = 1;
}
