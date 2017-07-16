#include "RedisClient.h"

std::string RedisClient::get(const std::string& key) {
	// Call GET command
	reply_ = (redisReply *)redisCommand(context_, "GET %s", key.c_str());

	// Check for errors
	if (reply_ == nullptr) {
		throw std::runtime_error("RedisClient: GET '" + key + "' failed.");
	} else if (reply_->type != REDIS_REPLY_STRING) {
		freeReplyObject(reply_);
		throw std::runtime_error("RedisClient: GET '" + key + "' returned non-string value.");
	}

	// Return value
	std::string value = reply_->str;
	freeReplyObject(reply_);
	return value;
}

void RedisClient::set(const std::string& key, const std::string& value) {
	// Call SET command
	reply_ = (redisReply *)redisCommand(context_, "SET %s %s", key.c_str(), value.c_str());

	// Check for errors
	if (reply_ == nullptr) {
		throw std::runtime_error("RedisClient: SET '" + key + "' '" + value + "' failed.");
	} else if (reply_->type == REDIS_REPLY_ERROR) {
		freeReplyObject(reply_);
		throw std::runtime_error("RedisClient: SET '" + key + "' '" + value + "' failed.");
	}

	freeReplyObject(reply_);
}

std::vector<std::string> RedisClient::get(const std::vector<std::string>& keys) {
	// Prepare key list
	for (const auto& key : keys) {
		redisAppendCommand(context_, "GET %s", key.c_str());
	}

	std::vector<std::string> values;
	for (size_t i = 0; i < keys.size(); i++) {
		int r = redisGetReply(context_, (void **)&reply_);
		if (r == REDIS_ERR) {
			throw std::runtime_error("RedisClient: Pipeline GET command failed.");
		} else if (reply_->type != REDIS_REPLY_STRING) {
			throw std::runtime_error("RedisClient: Pipeline GET command returned non-string value.");
		}

		values.push_back(reply_->str);
		freeReplyObject(reply_);
	}
	return values;
}

void RedisClient::set(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key list
	for (const auto& keyval : keyvals) {
		redisAppendCommand(context_, "SET %s %s", keyval.first.c_str(), keyval.second.c_str());
	}

	for (size_t i = 0; i < keyvals.size(); i++) {
		int r = redisGetReply(context_, (void **)&reply_);
		if (r == REDIS_ERR) {
			throw std::runtime_error("RedisClient: Pipeline SET command failed.");
		} else if (reply_->type == REDIS_REPLY_ERROR) {
			throw std::runtime_error("RedisClient: Pipeline SET command failed.");
		}

		freeReplyObject(reply_);
	}
}

std::vector<std::string> RedisClient::mget(const std::vector<std::string>& keys) {
	// Prepare key list
	std::vector<const char *> argv = {"MGET"};
	for (const auto& key : keys) {
		argv.push_back(key.c_str());
	}

	// Call MGET command with variable argument formatting
	reply_ = (redisReply *)redisCommandArgv(context_, argv.size(), &argv[0], nullptr);

	// Check for errors
	if (reply_ == nullptr) {
		throw std::runtime_error("RedisClient: MGET command failed.");
	} else if (reply_->type != REDIS_REPLY_ARRAY) {
		freeReplyObject(reply_);
		throw std::runtime_error("RedisClient: MGET command failed.");
	}

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < reply_->elements; i++) {
		if (reply_->element[i]->type != REDIS_REPLY_STRING) {
			freeReplyObject(reply_);
			throw std::runtime_error("RedisClient: MGET command returned non-string values.");
		}

		values.push_back(reply_->element[i]->str);
	}

	// Return values
	freeReplyObject(reply_);
	return values;
}

void RedisClient::mset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key-value list
	std::vector<const char *> argv = {"MSET"};
	for (const auto& keyval : keyvals) {
		argv.push_back(keyval.first.c_str());
		argv.push_back(keyval.second.c_str());
	}

	// Call MSET command with variable argument formatting
	reply_ = (redisReply *)redisCommandArgv(context_, argv.size(), &argv[0], nullptr);

	// Check for errors
	if (reply_ == nullptr) {
		throw std::runtime_error("RedisClient: MSET command failed.");
	} else if (reply_->type == REDIS_REPLY_ERROR) {
		freeReplyObject(reply_);
		throw std::runtime_error("RedisClient: MSET command failed.");
	}

	freeReplyObject(reply_);
}

static inline Eigen::MatrixXd decodeEigenMatrixWithDelimiters(const std::string& str,
	char col_delimiter, char row_delimiter, const std::string& delimiter_set,
	size_t idx_row_end = std::string::npos)
{
	// Count number of columns
	size_t num_cols = 0;
	size_t idx = 0;
	size_t idx_col_end = str.find_first_of(row_delimiter);
	while (idx < idx_col_end) {
		// Skip over extra whitespace
		idx = str.find_first_not_of(' ', idx);
		if (idx >= idx_col_end) break;

		// Find next delimiter
		idx = str.find_first_of(col_delimiter, idx + 1);
		++num_cols;
	}
	if (idx > idx_col_end) idx = idx_col_end;

	// Count number of rows
	size_t num_rows = 1;  // First row already traversed
	while (idx < idx_row_end) {
		// Skip over irrelevant characters
		idx = str.find_first_not_of(row_delimiter, idx);
		if (idx >= idx_row_end) break;

		// Find next delimiter
		idx = str.find_first_of(row_delimiter, idx + 1);
		++num_rows;
	}

	// Check number of rows and columns
	if (num_cols == 0)
		throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
	if (num_rows == 1) {
		// Convert to vector
		num_rows = num_cols;
		num_cols = 1;
	}

	// Parse matrix
	Eigen::MatrixXd matrix(num_rows, num_cols);
	std::string str_local(str);
	for (char delimiter : delimiter_set) {
		std::replace(str_local.begin(), str_local.end(), delimiter, ' ');
	}
	std::stringstream ss(str_local);
	for (size_t i = 0; i < num_rows; ++i) {
		for (size_t j = 0; j < num_cols; ++j) {
			std::string val;
			ss >> val;
			try {
				matrix(i,j) = std::stod(val);
			} catch (const std::exception& e) {
				throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
			}
		}
	}

	return matrix;
}

Eigen::MatrixXd RedisClient::decodeEigenMatrixString(const std::string& str) {
	return decodeEigenMatrixWithDelimiters(str, ' ', ';', ";");
}

Eigen::MatrixXd RedisClient::decodeEigenMatrixJSON(const std::string& str) {
	// Find last nested row delimiter
	size_t idx_row_end = str.find_last_of(']');
	if (idx_row_end != std::string::npos) {
		size_t idx_temp = str.substr(0, idx_row_end).find_last_of(']');
		if (idx_temp != std::string::npos) idx_row_end = idx_temp;
	}
	return decodeEigenMatrixWithDelimiters(str, ',', ']', ",[]", idx_row_end);
}

