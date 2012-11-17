#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
using std::cout;
using std::endl;
using std::vector;
using std::string;

vector<string>  g_cliArguments;


string          getTypedClassLoaderTemplate();
string          getTypedClassLoaderTemplateWithBaseSet();
void            generateFile(string filename, string contents);

void            processUserCommand();
vector<string>  getAvailablePlugins();
bool            loadPlugin(std::string plugin_name);

void            handleFindPluginRequest();
void            handleListPluginsRequest();
void            handleLoadPluginRequest();

string          whereIsAPlugin(std::string plugin_name);
vector<string>  getCLIArguments();
void            setCLIArguments(int argc, char* argv[]);

int main(int argc, char* argv[])
/*****************************************************************************/
{
	if(argc < 2)
	{
		cout << "Error: Not enough arguments.";
		return -1;
	}

	processUserCommand();

   return 0;
}

void processUserCommand()
/*****************************************************************************/
{
	vector<string> args = getCLIArguments();
	string cmd = args.at(0);
	assert(cmd == "plugin_tool");
	string verb = args.at(1);
	if (verb == "find")
		handleFindPluginRequest();
	else if (verb == "list")
		handleListPluginsRequest();
	else if(verb == "load")
		handleLoadPluginRequest();
	else
		cout << "Error: Unknown verb for plugin_tool, available verbs are 'load', 'list', and 'find'." << endl;
}


vector<string> getAvailablePlugins()
/*****************************************************************************/
{
	pluginlib::ClassLoader<

}


bool loadPlugin(std::string plugin_name)
/*****************************************************************************/
{

}


string whereIsAPlugin(std::string plugin_name)
/*****************************************************************************/
{
}


vector<string>  getCLIArguments()
/*****************************************************************************/
{
	return(g_cliArguments);
}

void setCLIArguments(int argc, char* argv[])
/*****************************************************************************/
{

	g_cliArguments = vector<string>(argc);
	for(int c = 0; c < argc; c++)
		g_cliArgumentss.at(c) = string(argv[c]);
}

string getTypedClassLoaderTemplate()
/*****************************************************************************/
{
	ifstream file;
	file.open("typed_class_loader_template.template");
	if(!file)
	{
		cout << "Error: Cannot find file 'typed_class_loader_template.template' to generate class loader";
	}

	string contents;
	while(!file.eof())
		contents.push_back(file.at(c));
}

string getTypedClassLoaderTemplateWithBaseSet()
/*****************************************************************************/
{

}

void generateFile(string filename, string contents)
/*****************************************************************************/
{
	ofstream file;
	file.open(filename.c_str());
	file << contents;
	file.close();
}