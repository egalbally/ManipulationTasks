/**
 * RedisClient.cpp
 *
 * Author: Toki Migimatsu
 * Created: April 2017
 */

#include "RedisClient.h"
#include <iostream>
#include <sstream>

void RedisClient::connect(const std::string& hostname, const int port,
	                      const struct timeval& timeout) {
	// Connect to new server
	context_.reset(nullptr);
	redisContext *c= redisConnectWithTimeout(hostname.c_str(), port, timeout);
	std::unique_ptr<redisContext, redisContextDeleter> context(c);

	// Check for errors
	if (!context)
		throw std::runtime_error("RedisClient: Could not allocate redis context.");
	if (context->err)
		throw std::runtime_error("RedisClient: Could not connect to redis server: " + std::string(context->errstr));

	// Save context
	context_ = std::move(context);
}

std::unique_ptr<redisReply, redisReplyDeleter> RedisClient::command(const char *format, ...) {
	va_list ap;
	va_start(ap, format);
	redisReply *reply = (redisReply *)redisvCommand(context_.get(), format, ap);
	va_end(ap);
	return std::unique_ptr<redisReply, redisReplyDeleter>(reply);
}

void RedisClient::ping() {
	auto reply = command("PING");
	std::cout << std::endl << "RedisClient: PING " << context_->tcp.host << ":" << context_->tcp.port << std::endl;
	if (!reply) throw std::runtime_error("RedisClient: PING failed.");
	std::cout << "Reply: " << reply->str << std::endl << std::endl;
}

std::string RedisClient::get(const std::string& key) {
	// Call GET command
	auto reply = command("GET %s", key.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR || reply->type == REDIS_REPLY_NIL)
		throw std::runtime_error("RedisClient: GET '" + key + "' failed.");
	if (reply->type != REDIS_REPLY_STRING)
		throw std::runtime_error("RedisClient: GET '" + key + "' returned non-string value.");

	// Return value
	return reply->str;
}

void RedisClient::set(const std::string& key, const std::string& value) {
	// Call SET command
	auto reply = command("SET %s %s", key.c_str(), value.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: SET '" + key + "' '" + value + "' failed.");
}

void RedisClient::del(const std::string& key) {
	// Call DEL command
	auto reply = command("DEL %s", key.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: DEL '" + key + "' failed.");
}

std::vector<std::string> RedisClient::pipeget(const std::vector<std::string>& keys) {
	// Prepare key list
	for (const auto& key : keys) {
		redisAppendCommand(context_.get(), "GET %s", key.c_str());
	}

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < keys.size(); i++) {
		redisReply *r;
		if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
			throw std::runtime_error("RedisClient: Pipeline GET command failed for key:" + keys[i] + ".");
		
		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type != REDIS_REPLY_STRING)
			throw std::runtime_error("RedisClient: Pipeline GET command returned non-string value for key: " + keys[i] + ".");

		values.push_back(reply->str);
	}
	return values;
}

void RedisClient::pipeset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key list
	for (const auto& keyval : keyvals) {
		redisAppendCommand(context_.get(), "SET %s %s", keyval.first.c_str(), keyval.second.c_str());
	}

	for (size_t i = 0; i < keyvals.size(); i++) {
		redisReply *r;
		if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
			throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");

		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type == REDIS_REPLY_ERROR)
			throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");
	}
}

std::vector<std::string> RedisClient::mget(const std::vector<std::string>& keys) {
	// Prepare key list
	std::vector<const char *> argv = {"MGET"};
	for (const auto& key : keys) {
		argv.push_back(key.c_str());
	}

	// Call MGET command with variable argument formatting
	redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
	std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

	// Check for errors
	if (!reply || reply->type != REDIS_REPLY_ARRAY)
		throw std::runtime_error("RedisClient: MGET command failed.");

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < reply->elements; i++) {
		if (reply->element[i]->type != REDIS_REPLY_STRING)
			throw std::runtime_error("RedisClient: MGET command returned non-string values.");

		values.push_back(reply->element[i]->str);
	}
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
	redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
	std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: MSET command failed.");
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

