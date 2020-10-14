using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace AR.Drone.Util
{
    class ConfigGenerator
    {
        internal string _inputFile;
        internal string _outputPath;

        public ConfigGenerator(string inputFile, string outputPath)
        {
            _inputFile = inputFile;
            _outputPath = outputPath;
        }

        public void GenerateConfigClasses()
        {
            string contentTemplate = "using AR.Drone.Control.Command;\nusing AR.Drone.Control;\nusing AR.Drone.Configuration.Section;\nusing System;\nusing System.Collections.Generic;\nusing System.Linq;\nusing System.Text;\nusing System.Threading.Tasks;\n\nnamespace AR.Drone.Configuration.Section\n{{\n\tclass {0} : AbstractConfigSection\n\t{{\n\n{1}\n\n{2}}}}}";
            string constructorTemplate = "\t\tpublic {0}(ref CommandSender commandSender)\n\t\t\t:base(ConfigSection.{1}, ref commandSender) {{ }}";
            string data = System.IO.File.ReadAllText(@_inputFile);


            string reg = "([a-zA-Z]*):([a-zA-Z_0-9]*) = ([^!\n]*)([!]?)[\n]";
            MatchCollection result = Regex.Matches(data, reg);

            Dictionary<string, string> sectionDictionary = new Dictionary<string, string>();


            foreach (Match match in result)
            {
                string section = match.Groups[1].Value;
                string key = match.Groups[2].Value;
                string value = match.Groups[3].Value;
                bool readOnly = match.Groups[4].Value.Contains("!");

                AddToSectionString(section, GetFieldString(key, value, readOnly), ref sectionDictionary);
            }

            string className, content, constructor;
            foreach (var section in sectionDictionary)
            {
                className = getClassName(section.Key);
                constructor = string.Format(constructorTemplate, className, section.Key.ToUpper());
                content = string.Format(contentTemplate, className, constructor, section.Value);

                System.IO.File.WriteAllText(@_outputPath + className + ".cs", content);

                //content += section.Key.ToUpper() + ",\n";

                //content += string.Format("case ConfigSection.{0}:\n\t{1}.Add(key, value);\n\tbreak;\n", section.Key.ToUpper(), section.Key);
                //System.IO.File.WriteAllText(@"C:\Users\d059315\Documents\DHBW\Studienarbeit\AR Drone Source\C#\Enum.txt", content);
            }
        }

        private string getClassName(string section)
        {
            var className = char.ToUpper(section[0]) + section.Substring(1);
            return string.Format("{0}ConfigSection", className);
        }

        private void AddToSectionString(string section, string content, ref Dictionary<string, string> sectionDictionary)
        {
            string result;
            if (!sectionDictionary.TryGetValue(section, out result))
            {
                result = "";
            }
            else
            {
                sectionDictionary.Remove(section);
            }

            result += content;
            sectionDictionary.Add(section, result);
        }

        private string GetFieldString(string key, string value, bool readOnly)
        {
            string methodeName;
            string type = GetSuitableType(value, out methodeName);

            if (readOnly)
            {
                return string.Format("\t\tinternal {0} {1} \n \t\t{{\n\t\t\tget {{ return {2}(\"{3}\"); }}\n \t\t}} \n\n ", type, key, methodeName, key);
            }
            else
            {
                return string.Format("\t\tinternal {0} {1} \n \t\t{{\n\t\t\tget {{ return {2}(\"{3}\"); }}\n\t\t\tset {{ Set(\"{4}\", value); }}\n \t\t}} \n\n ", type, key, methodeName, key, key);
            }

        }

        private string GetSuitableType(string s, out string methodeName)
        {
            int intResult;
            if (Int32.TryParse(s, out intResult))
            {
                methodeName = "GetIntValue";
                return "int";
            }

            float floatResult;
            if (float.TryParse(s, out floatResult))
            {
                methodeName = "GetFloatValue";
                return "float";
            }

            methodeName = "GetStringValue";
            return "string";
        }
    }
}