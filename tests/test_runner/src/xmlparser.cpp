/**
 * @brief       XML parser class
 * @file        xmlparser.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "../include/xmlparser.h"
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

xmlNodePtr XMLParser :: findNodeByName(xmlNodePtr srchNode, const xmlChar * nodename)
{
    xmlNodePtr node = srchNode;
    if(node == NULL){
        fprintf(stderr, "Document is empty\n");
        return NULL;
    }

    while(node != NULL){
        if(!xmlStrcmp(node->name, nodename)){
            return node; 
        }
        else if(node->children != NULL){
			xmlNodePtr intNode =  findNodeByName(node->children, nodename); 
			if(intNode != NULL) {
				return intNode;
			}
        }
        node = node->next;
    }
	//printf("Coming up null for %s\n",nodename);
    return NULL;
}

std::string XMLParser :: getNodeValue(const xmlChar * nodename)
{
	std::string ret="";

	//Reset to the root node.
	rootNode=xmlDocGetRootElement(doc);
	xmlNodePtr locatedData=findNodeByName(rootNode, nodename);
	if (locatedData!=NULL)
	{
		xmlChar *key = xmlNodeListGetString(doc, locatedData->xmlChildrenNode, 1);
		ret.append((const char*)key);
		xmlFree(key);
	}
	return ret;
}

XMLParser :: XMLParser(const char *filename) {
	openXMLFile(filename);
}

XMLParser :: ~XMLParser() {
	//Free XML objects
	xmlFreeDoc(doc);

	/*
	* Cleanup function for the XML library.
	*/
	xmlCleanupParser();
}

void XMLParser :: getXMLVector(const char *vectorName, std::vector<std::string> &vec)
{
	std::string str = getNodeValue((const xmlChar *)vectorName);

	//printf("Result for %s: %s\n",vectorName,str.c_str());
	/*Tokenize on closing bracket, if present. Ideally, would do "],", but multiple chars not supported this way. 
	So, will remove, but need to get rid of opening brackets and leading commas.*/
	if (str.find("]") != std::string::npos) 
	{
    	boost::char_separator<char> sep("]");
		boost::tokenizer<boost::char_separator<char>> tokens(str, sep);
		int tokenCount=0;
		for (auto t : tokens) {
			//std::cout << t << "." << std::endl;

			//There is only the opening bracket to erase. Else, there is also a comma, and that too must go.
			if (tokenCount==0)
			{
				t.erase(0, 1);
			}
			else
			{
				t.erase(0, 2);
			}
			tokenCount++;
			vec.push_back(t);
		}
	}

	//No closing bracket, likely just a list of singular entries (a list of numbers, etc.)
	else
	{
		boost::char_separator<char> sep(",");
		boost::tokenizer<boost::char_separator<char>> tokens(str, sep);
		for (auto t : tokens) {
			//std::cout <<"Singular case: "<< t << "." << std::endl;
			vec.push_back(t);
		}
	}
}

void XMLParser :: openXMLFile(const char *filename)
{
	/* doc = xmlParseFile(filename);

	if (doc == NULL ) {
        fprintf(stderr,"Unable to open XML file %s. \n",filename);
        return;
    }
	rootNode=xmlDocGetRootElement(doc); */

	//Because the data files can have large nodes, set option of XML_PARSE_HUGE.
	doc = xmlReadFile(filename, NULL, XML_PARSE_HUGE);
	rootNode=xmlDocGetRootElement(doc);
}