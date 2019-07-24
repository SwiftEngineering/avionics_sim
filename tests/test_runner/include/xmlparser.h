#ifndef XMLPARSER_H_
#define XMLPARSER_H_

/**
 * @brief       XML parser class
 * @file        xmlparser.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <libxml/xmlreader.h>

class XMLParser {
public:

	//Constructor accepting XML filename.
	XMLParser(const char *filename);

	//Default constructor
	XMLParser() {};

	//Destructor
	~XMLParser();

	// Function to retrieve node data into a vector of strings.  
	///
	/// \brief      Assuming a node with comma delimited collection of bracketed strings, read each string into a vector. 
	///
	/// \details    N/A
	/// \param[in]  vectorName (Name of node in XML file)
	/// \param[inout]  vec (Reference to vector that will hold the vector of strings)
	/// \return     N/A
	///
	void getXMLVector(const char *vectorName, std::vector<std::string> &vec);

	// Function to open integration test XML data.   
	///
	/// \brief      N/A 
	///
	/// \details    N/A
	/// \param[in]  filename (XML filename)
	/// \return     N/A
	///
	void openXMLFile(const char *filename);
private:

	// Function to get node's value as a string.   
	///
	/// \brief      N/A 
	///
	/// \details    N/A
	/// \param[in]  nodename (name of node)
	/// \return     N/A
	///
	std::string getNodeValue(const xmlChar * nodename);

	// Function to find node in the integration test XML data.   
	///
	/// \brief      N/A 
	///
	/// \details    N/A
	/// \param[in]  srchNode (reference to XML node ptr to search)
	/// \param[in]  nodename (name of node to search for)
	/// \return     N/A
	///
	xmlNodePtr findNodeByName(xmlNodePtr srchNode, const xmlChar * nodename);

	//XML Document pointer
	xmlDocPtr doc;

	//XML Text Reader Pointer
	xmlTextReaderPtr reader;

	//Pointer to root node
	xmlNodePtr rootNode;
};
#endif
