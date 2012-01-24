/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef _CMD_ARGUMENTS_H
#define _CMD_ARGUMENTS_H

#include <netctrl/version.h>
#include <string>
#include <vector>
#include "SimpleOpt.h"

/// Possible model types handled by the application
typedef enum {
    LIU_MODEL, SWITCHBOARD_MODEL
} ModelType;

/// Possible operation modes for the application
typedef enum {
    MODE_DRIVER_NODES, MODE_STATISTICS, MODE_SIGNIFICANCE,
    MODE_CONTROL_PATHS
} OperationMode;

/// Parses the command line arguments of the main app
class CommandLineArguments {
private:
	/// String storing the name of the executable used to start the program
	std::string m_executableName;

    /// String storing the version number of the application
    std::string m_versionNumber;

    /// A vector of command line option specifications
    std::vector<SimpleOpt::CSimpleOpt::SOption> m_options;

public:
    /********************/
    /* Basic parameters */
    /********************/

	/// Name of the input file
	std::string inputFile;

    /// Verbosity level
    int verbosity;

    /// Name of the output file
    std::string outputFile;

    /// Model type used by the application
    ModelType modelType;

    /// Operation mode of the application (i.e. what we are going to calculate)
    OperationMode operationMode;

    /// Flag to denote whether we are using the edge-based measure for SBD
    bool useEdgeMeasure;

public:
	/// Constructor
	CommandLineArguments(const std::string programName = "netctrl",
            const std::string version = NETCTRL_VERSION_STRING);

    /// Adds an option to the list of command line options
    void addOption(int id, const char* option, SimpleOpt::ESOArgType type,
                   const char* longOption = 0);
    
    /// Handles the option with the given ID and argument
    /**
     * \return  zero if everything is OK, an exit code otherwise
     */
    int handleOption(int id, const std::string& arg);

    /// Parses the command line arguments
    void parse(int argc, char** argv);

	/// Shows a help message on the given stream
	void showHelp(std::ostream& os) const;

protected:
    /// Shows the "General options" section from the help message
    void showGeneralOptionsHelp(std::ostream& os) const;
};

#endif     // _CMD_ARGUMENTS_BASE_H
