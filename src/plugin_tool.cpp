#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <dlfcn.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;

vector<string>  g_cli_arguments;
void*           g_class_loader_library_handle = NULL;

void            processUserCommand();
vector<string>  getAvailablePlugins();
bool            loadPlugin(std::string plugin_name);

void            handleFindPluginRequest();
void            handleListPluginsRequest();
void            handleLoadPluginRequest();

string          whereIsAPlugin(std::string plugin_name);

//Command line arguments
vector<string>  getCLIArguments();
void            setCLIArguments(int argc, char* argv[]);
string          commandVerb(){return(getCLIArguments().at(1));}
string          baseClass(){return(getCLIArguments().at(2));}
string          baseClassHeader(){return(getCLIArguments().at(3));}
string          packageName(){return(getCLIArguments().at(4));}
string          pluginName(){return(getCLIArguments().at(5));}

//Typed Class Loader Interface (shared object)
void            generateAndLoadTypedPluginInterface();
string          getTypedClassLoaderTemplate();
string          getTypedClassLoaderTemplateWithBaseSet();
void            generateFile(string filename, string contents);

std::string              callCommandLine(const char* cmd);
std::vector<std::string> parseToStringVector(std::string newline_delimited_str);
std::string              getPluginlibSharedFolder();

std::string              generatedSharedLibrary(){return "libTypedPluginInterface.so";}
std::string              generatedObjFile(){return "typedPluginInterface.o";}
std::string              generatedCppFile(){return "typedPluginInterface.cpp";}
std::string              templateCppFileWithoutExtension(){return "typedPluginInterface";}
std::string              templateCppFile(){return templateCppFileWithoutExtension() + ".cpp";}
std::string              templateCppFileAbsolutePath(){return getPluginlibSharedFolder() + "/" + templateCppFile();}

int main(int argc, char* argv[])
/*****************************************************************************/
{
	setCLIArguments(argc, argv);

	cout << "plugin_tool - A command line tool for pluginlib testing" << endl;
	cout << "-------------------------------------------------------" << endl;
	if(argc < 5)
	{
		cout << "Error: Not enough arguments. Usage: plugin_tool <BASE CLASS> <BASE_CLASS_HEADER> <PACKAGE_NAME> <find [class_name] | list | load [class_name]>";
		return -1;
	}

	cout << "Base class = " << baseClass() << endl;
	cout << "Package name = " << packageName() << endl << endl;

	generateAndLoadTypedPluginInterface();
	processUserCommand();

   return 0;
}


void generateAndLoadTypedPluginInterface()
/*****************************************************************************/
{
	cout << "Generating typed plugin interface cpp..." << endl;
	string code = getTypedClassLoaderTemplateWithBaseSet();
	cout << "***************************************************" << endl;
	cout << code << endl;
	cout << "***************************************************" << endl;

	cout << "Outputting to file " << templateCppFile() << "..." << endl;
	generateFile(generatedCppFile(), code);

	cout << "Building interface shared object..." << endl;
	string cmd1 = "g++ -fPIC -DBASE_CLASS=" + baseClass() + " -c " + generatedCppFile();
	string cmd2 = "g++ -shared -o " + generatedSharedLibrary() + " " + generatedObjFile();

	cout << "Command 1 = " << cmd1 << endl;
	cout << "Command 2 = " << cmd2 << endl;

	if(-1 == system(cmd1.c_str()))
	{
		cout << "Error: Failed to compile interface." << endl;
		exit(-1);
	}
	if(-1 == system(cmd2.c_str()))
	{
		cout << "Error: Failed to build shared object." << endl;
		exit(-1);
	}

	cout << "Loading shared object into memory." << endl;
	g_class_loader_library_handle = dlopen("libTypedPluginInterface.so", RTLD_LAZY);
	if(g_class_loader_library_handle)
		cout << "Shared object successfully loaded into memory." << endl;		
	else
	{
		cout << "Error: Failed to load shared object into memory." << endl;
		exit(-1);
	}
}

template <typename T> T getPluginFunction(const std::string& function_name)
/*****************************************************************************/
{
	void* ptr = dlsym(g_class_loader_library_handle, function_name.c_str());
	return((T)(ptr));
}

void handleFindPluginRequest()
/*****************************************************************************/
{
	//string whereIsPluginLocated(const string& package_name, const string& class_name)
	typedef std::string (*WhereIsFunc)(const string&, const string&);
	WhereIsFunc f = getPluginFunction<WhereIsFunc>("whereIsPluginLocated");
	
	cout << "Attempting to find plugin " << pluginName() << " exported from package " << packageName() << "..." << endl;

	if(f)
		cout << "Plugin " << pluginName() << " is located in library " << f(packageName(), pluginName()) << endl;
	else
	{
		cout << "Error: Could not find function 'whereIsPluginLocated' in shared object." << endl;
		exit(-1);
	}
}

void handleListPluginsRequest()
/*****************************************************************************/
{
	//  std::vector<std::string> availablePlugins(const string& package_name)
	typedef std::vector<std::string> (*ListFunc)(const string&);
	ListFunc f = getPluginFunction<ListFunc>("availablePlugins");

	cout << "The following plugins are available in package " << packageName() << ":" << endl;
	if(f)
	{

		std::vector<std::string> plugins = f(packageName());
		for(unsigned int c = 0; c < plugins.size(); c++)
			cout << plugins.at(c) << endl;
	}
	else
	{
		cout << "Error: Could not find function 'availablePlugins' in shared object." << endl;
		exit(-1);
	}
}

void handleLoadPluginRequest()
/*****************************************************************************/
{
	//  bool loadPlugin(const string& package_name, const string& class_name)  
	typedef bool (*LoadPluginFunc)(const string&, const string&);
	LoadPluginFunc f = getPluginFunction<LoadPluginFunc>("loadPlugin");
	string plugin_name = getCLIArguments().at(4);
	cout << "Attempting to find plugin " << plugin_name << "..." << endl;
	if(f)
	{
		if(f(packageName(), plugin_name))
			cout << "Opened plugin successfully :)" << endl;
		else
		{
			cout << "Error: Plugin did not open :(" << endl;
			exit(-1);
		}
	}
	else
	{
		cout << "Error: Could not find function 'loadPlugin' in shared object." << endl;
		exit(-1);
	}	
}

vector<string>  getCLIArguments()
/*****************************************************************************/
{
	return(g_cli_arguments);
}

void generateFile(string filename, string contents)
/*****************************************************************************/
{
	std::ofstream file;
	file.open(filename.c_str());
	file << contents;
	file.close();
}

string getTypedClassLoaderTemplate()
/*****************************************************************************/
{
	std::ifstream file;
   string path = getPluginlibSharedFolder() + "/typed_class_loader_template.cpp";
	file.open(path.c_str());
	if(!file)
	{
		cout << "Error: Cannot find file " + path + " to generate class loader";
		exit(-1);
	}

	string contents;
	while(!file.eof())
	{
		char c;
		file.get(c);
		contents.push_back(c);
		cout << c;
	}		

	return contents;
}

string getTypedClassLoaderTemplateWithBaseSet()
/*****************************************************************************/
{
	string class_template = getTypedClassLoaderTemplate();
	string class_with_type_set;
	for(unsigned int c = 0; c < class_template.size(); c++)
	{
		if(class_template.at(c) == '$')
			class_with_type_set += baseClass();
		else if(class_template.at(c) == '@')
			class_with_type_set += "#include \"" + baseClassHeader() + "\"";
		else
			class_with_type_set.push_back(class_template.at(c));
	}
	return(class_with_type_set);
}

void processUserCommand()
/*****************************************************************************/
{
	vector<string> args = getCLIArguments();
	string cmd = args.at(0);
	assert(cmd == "plugin_tool");

	string verb = commandVerb();

	if (verb == "find")
		handleFindPluginRequest();
	else if (verb == "list")
		handleListPluginsRequest();
	else if(verb == "load")
		handleLoadPluginRequest();
	else
		cout << "Error: Unknown verb for plugin_tool, available verbs are 'load', 'list', and 'find'." << endl;
}

void setCLIArguments(int argc, char* argv[])
/*****************************************************************************/
{
	g_cli_arguments = vector<string>(argc);
	for(int c = 0; c < argc; c++)
		g_cli_arguments.at(c) = string(argv[c]);
}

std::string callCommandLine(const char* cmd)
/***************************************************************************/
{
	FILE* pipe = popen(cmd, "r");
	if (!pipe)
	  return "ERROR";
	char buffer[128];
	std::string result = "";
	while(!feof(pipe))
	{
	  if(fgets(buffer, 128, pipe) != NULL)
		  result += buffer;
	}
	pclose(pipe);
	return result;
}

std::vector<std::string> parseToStringVector(std::string newline_delimited_str)
/***************************************************************************/
{
 std::string next;
 std::vector<std::string> parse_result;
 for(unsigned int c = 0; c < newline_delimited_str.size(); c++)
 {
   char ch = newline_delimited_str.at(c);
   if(ch == '\n')
   {
     parse_result.push_back(next);
     next = "";
   }
   else
     next.push_back(ch);
 }
 return(parse_result);
}

std::string getPluginlibSharedFolder()
/***************************************************************************/
{
   vector<std::string> allVals = parseToStringVector(callCommandLine("catkin_find pluginlib --share"));
   assert(allVals.size() > 0);
   return(allVals.at(0));
}
