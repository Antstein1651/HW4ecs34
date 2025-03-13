#include "StringUtils.h"
#include <cstring> 
#include <cctype>

namespace StringUtils{

// Assignment = Returns a substring of the string str, allows for negative values as in
// python end == 0 means to include end of string
std::string Slice(const std::string &str, ssize_t start, ssize_t end) noexcept{
    // Replace code here

    ssize_t length = str.size(); // length of the string
    
    if (start < 0){
        start += length; // Adjusting if for C++
    }
    if (end < 0){
        end += length; // Adjusting if for C++ 
    } 

    if (start < 0) start = 0;
    if (end == 0) end = length; 
    if (end > length) end = length;
    if (start >= end) return ""; 

    return str.substr(start, end - start);
}

// Assignment: Returns the capitalized string as in python
std::string Capitalize(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }

    std::string result = str;  // capital_result will be the string with the first letter capitalized.

    for (int i = 0; (i < result.size()); i++){ // iterates through each character in the string
        result[i] = std::tolower(result[i]); // lowers each character
    }
    result[0] = std::toupper(result[0]); // Makes the first letter capital

    return result;
}

// Assignment: Returns the upper-case strings as in python
// Status: worked on
std::string Upper(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }
    
    std::string result = str; // Creates a copy to modify

    for (int i = 0; (i < result.size()); i++){ // Iterates through each character in the string
        result[i] = std::toupper(result[i]); // Makes each character upper case
    }

    return result; // return the result
}

// Assignment: Returns the lower-case strings as in python
// Status: worked on
std::string Lower(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }
    
    std::string result = str; // Creates a copy to modify

    for (int i = 0; (i < result.size()); i++){ // Iterates through each character in the string
        result[i] = std::tolower(result[i]); // Makes each character lower case
    }

    return result; // return the result
}

// Assignment: Returns the left stripped strings,
// (white space characters are removed from left, right or both).
std::string LStrip(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }

    size_t first = 0; // first = the first actual character in the string, non-whitespace
    while (first < str.size() && std::isspace(static_cast<unsigned char>(str[first]))) {
        ++first; // Increments first by one very iteration, in order to find the first actual character
    }

    // Return the substring from the first non-whitespace character to the end
    return str.substr(first);
}

// Assignment: Returns the right stripped strings,
// (white space characters are removed from left, right or both).
std::string RStrip(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }
    size_t last = str.size(); // last = last character in string
    while (last > 0 && std::isspace(static_cast<unsigned char>(str[last - 1]))) {
        --last;
    }

    // Return the substring from the beginning to the last non-whitespace character
    return str.substr(0, last);
}

// Assignment: Returns the both stripped strings,
// (white space characters are removed from left, right or both).
std::string Strip(const std::string &str) noexcept{

    if (str.empty()) { // Checking if the string is empty
        return str; // if empty it returns the string back
    }
    size_t last = str.size(); // last = last character in string
    while (last > 0 && std::isspace(static_cast<unsigned char>(str[last - 1]))) {
        --last;
    }
    size_t first = 0; // first = the first actual character in the string, non-whitespace
    while (first < str.size() && std::isspace(static_cast<unsigned char>(str[first]))) {
        ++first; // Increments first by one very iteration, in order to find the first actual character
    }

    return str.substr(first, last - first);
}
//Assignment: Returns the center/left/right justified strings
// width = width of the whole string essentially the length of how it should be
// fill = the character to fill the space with
// str = well, its just the string lol
std::string Center(const std::string &str, int width, char fill) noexcept{

    if (str.empty()|| width <= str.size()) { // Checking if the string is empty or if the width is less/equal of string size
        return str; // if empty it returns the string back
    }
    int extra = width - str.size(); // extra = the extra space that's not the actual string
    int left_extra = extra / 2; // side_extra = half of the extra space, so one side is the left and the other on the right
    int right_extra = extra - left_extra; // Right side gets the remaining 

    std::string result = str; // result = string we'll turn in
    result.insert(result.begin(), left_extra, fill); // inserting the beginning of the string with side_extra about of fill
    result.append(right_extra, fill); // appending amount of times on the end of the string(on the right)

    return result;
}

//Assignment: Returns the left justified strings
std::string LJust(const std::string &str, int width, char fill) noexcept{

    if (str.empty()|| width <= str.size()) { // Checking if the string is empty if the width is less/equal of string size
        return str; // if empty it returns the string back
    }

    int extra = width - str.size(); // extra = the extra space that's not the actual string

    std::string result = str; // result = string we'll turn in
    result.append(extra, fill); // appending amount of times on the end of the string(on the right)
  

    return result;
}

//Assignment: Returns the right justified strings
std::string RJust(const std::string &str, int width, char fill) noexcept{

    if (str.empty()|| width <= str.size()) { // Checking if the string is empty if the width is less/equal of string size
        return str; // if empty it returns the string back
    }
    int extra = width - str.size(); // extra = the extra space that's not the actual string
    std::string result = str; // result = string we'll turn in
    result.insert(result.begin(), extra, fill); // inserting the beginning of the string with side_extra about of fill
  
    return result;
}
// Assignment =  Returns the string str with all instances of old replaced with rep
std::string Replace(const std::string &str, const std::string &old, const std::string &rep) noexcept{
    
    if (str.empty() || old.empty()) { // Checking if the string is empty, or if the old is empty
        return str; // if empty it returns the string back
    }
    std::string result = str; // result = string that will be modified and returned
    size_t index = 0; // index = position where the old will be searched from

    while ((index = result.find(old, index)) != std::string::npos) { // starting from the beginning of the string, it looks for old
    result.replace(index, old.length(), rep); // Replaces old with rep
    index += rep.length(); // moves past it now that we replaced it, and iterates again
    }

    return result;
}

// Assignment = Splits the string up into a vector of strings based on splt parameter, if
// splt parameter is empty string, then split on white space
std::vector<std::string> Split(const std::string &str, const std::string &splt) noexcept{
    std::vector<std::string> splitsub;
    size_t start = 0, end;
    while ((end = str.find(splt, start)) != std::string::npos) { // find where we want to split the string
        splitsub.push_back(str.substr(start, end - start)); //create gap in the string 
        start = end + splt.length(); //move to the next part of the string
    }
    splitsub.push_back(str.substr(start)); // add the new string pack on 
    return splitsub;
}

// Assignment = Joins a vector of strings into a single string
std::string Join(const std::string &str, const std::vector< std::string > &vect) noexcept{

    if (vect.empty()) { // If the vector of strings is empty, then there's nothin to put as a string
        return "";
    }
    std::string result; // this will hold all the strings in the vector in one string.

    for (size_t i = 0; i < vect.size(); ++i) { //Iterates through each string in the vector
        result += vect[i]; // adds the string into our result single string
        if (i < vect.size() - 1) { 
            result += str;
        }
    }
    return result; // returns our single string
}

// Assignment = Replaces tabs with spaces aligning at the tabstops
// str = string
// tabsize = the size of the tab??
std::string ExpandTabs(const std::string &str, int tabsize) noexcept{
    std::string result;
    for (char c : str) {
        if (c == '\t') {
            result.append(tabsize - result.size() % tabsize, ' '); //find tabs in the string and replace them with tabsize spaces
        } else {
            result.push_back(c); //move the current character to the next space
        }
    }
    return result;
}

// Assignment = Calculates the Levenshtein distance (edit distance) between the two strings
int EditDistance(const std::string &left, const std::string &right, bool ignorecase) noexcept{
    size_t len1 = left.size(), len2 = right.size();
    std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1)); // create a 2D vector for the distances

    for (size_t i = 0; i <= len1; ++i) {
        for (size_t j = 0; j <= len2; ++j) {
            if (i == 0) {
                dp[i][j] = j; // find the cost of adding the j characters
            } else if (j == 0) {
                dp[i][j] = i; //find the cost of removing the i characters
            } else {
                int cost = (ignorecase ? (tolower(left[i - 1]) != tolower(right[j - 1])) 
                                       : (left[i - 1] != right[j - 1])) ? 1 : 0;
                
                dp[i][j] = std::min(std::min(dp[i - 1][j] + 1,   // delete the current cost
                                             dp[i][j - 1] + 1),  // insert the new cost
                                             dp[i - 1][j - 1] + cost); // substitude the distance plus the cost for the strings
            }
        }
    }
    return dp[len1][len2];
}

};