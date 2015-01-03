/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef NETCTRL_ERRORS_H
#define NETCTRL_ERRORS_H

#include <exception>

namespace netctrl {

class not_supported_error : public std::runtime_error {

public:
	not_supported_error() : std::runtime_error("This operation is not supported") {}
	explicit not_supported_error(const char* message)
		: std::runtime_error(message) {}
	explicit not_supported_error(const std::string& message)
		: std::runtime_error(message) {}

	virtual ~not_supported_error() throw() {}
};

}          // end of namespace

#endif


