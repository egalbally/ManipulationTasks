/**
 * OptiTrackClient.h
 *
 * Author: Toki Migimatsu
 * Created: April 2017
 */

#ifndef OPTITRACK_CLIENT_H
#define OPTITRACK_CLIENT_H

// NatNetLinux
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>

// std
#include <vector>
#include <thread>
#include <chrono>
#include <fstream>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace OptiTrackServer {
	// Default framerate for CS225a OptiTrack system
	const int DEFAULT_FPS = 120;

	// Maximum framerate for CS225a OptiTrack system
	const int MAX_FPS = 240;

	// Default server IP for CS225a OptiTrack system
	const std::string DEFAULT_IP = "172.24.68.48";
}

class OptiTrackClient {

public:

	OptiTrackClient() {}
	~OptiTrackClient();

	/**
	 * Establish connection with OptiTrack server.
	 *
	 * To stream data over OptiTrack Motive, go to View > Data Streaming Pane
	 * and check Broadcast Frame Data.
	 *
	 * @param local_public_ip_address  Public IP address (not 127.0.0.1).
	 * @param server_ip_address        IP address specified under Network
	 *                                 Interface in Motive's Data Streaming pane.
	 *                                 Set to kOptiTrackServerIp225a by default.
	 * @return  True if connection is established, otherwise false.
	 */
	bool openConnection(const std::string& local_public_ip_address, const std::string& server_ip_address = OptiTrackServer::DEFAULT_IP);

	/**
	 * Close connection with OptiTrack server.
	 */
	void closeConnection();

	/**
	 * Open CSV file exported from OptiTrack Motive.
	 *
	 * @param filename  File to open relative to executable path
	 * @return  True if file successfully opens, otherwise false.
	 */
	bool openCsv(const std::string& filename);

	/**
	 * Close CSV file.
	 */
	void closeCsv();

	/**
	 * Try to retrieve a frame from the OptiTrack server.
	 *
	 * Will return the most recent frame to match the current expected
	 * timestamp. For CSV files, this means some frames may be skipped.
	 *
	 * @return  True if server responds with a frame, otherwise false.
	 */
	bool getFrame();

	/**
	 * Timestamp of current frame in seconds, starting at 0s.
	 */
	double frameTime() {
		return static_cast<double>(t_frame_) / fps_;
	};

	/**
	 * Frame number of current frame, starting from 0.
	 */
	size_t frameNum() {
		return t_frame_;
	}

	/**
	 * Framerate (default 120). Should be set to match OptiTrack Motive.
	 */
	int fps() {
		return fps_;
	}
	bool setFps(int fps);

	/**
	 * Positions of rigid bodies in current frame.
	 */
	std::vector<Eigen::Vector3f> pos_rigid_bodies_;

	/**
	 * Orientations of rigid bodies in current frame.
	 */
	std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> ori_rigid_bodies_;

	std::vector<Eigen::Vector3f> ori_rigid_bodies_ea_;

	/**
	 * Positions of single markers in current frame.
	 */
	std::vector<Eigen::Vector3f> pos_single_markers_;

private:

	enum MarkerType {
		RIGID_BODY_POSITION,
		RIGID_BODY_ORIENTATION,
		SINGLE_MARKER_POSITION,
		OTHER
	};

	// Private methods
	bool readNetworkFrame();
	bool readCsvFrame();
	void resetState();

	// Listener threads
	std::unique_ptr<CommandListener> command_listener_;
	std::unique_ptr<FrameListener> frame_listener_;

	// Socket descriptors
	int sd_data_;
	int sd_command_;

	// CSV variables
	std::ifstream csv_file_;
	std::vector<MarkerType> marker_types_;
	int num_single_marker_ids_;

	// Timer variables
	bool t_initialized_ = false;
	std::chrono::high_resolution_clock::time_point t_start_;
	size_t t_frame_start_ = 0;
	size_t t_frame_ = 0;
	size_t t_frame_next_ = 1;
	int fps_ = OptiTrackServer::DEFAULT_FPS;

};

#endif  // OPTITRACK_H
