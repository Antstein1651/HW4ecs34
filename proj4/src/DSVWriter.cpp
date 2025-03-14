#include "DSVWriter.h"
#include <vector>
#include <sstream>

struct CDSVWriter::SImplementation {
    std::shared_ptr<CDataSink> DSink;
    char DDelimiter;
    bool DQuoteAll;

    SImplementation(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall)
        : DSink(sink), DDelimiter(delimiter), DQuoteAll(quoteall) {}

    bool WriteRow(const std::vector<std::string>& row) {
        std::ostringstream line;
        bool first = true;

        for (const auto& field : row) {
            if (!first) {
                line << DDelimiter;
            }
            first = false;

            bool needQuotes = DQuoteAll || (field.find(DDelimiter) != std::string::npos || field.find('"') != std::string::npos);
            if (needQuotes) {
                line << '"';
                for (char ch : field) {
                    if (ch == '"') {
                        line << "\"\""; // Escape double quotes
                    } else {
                        line << ch;
                    }
                }
                line << '"';
            } else {
                line << field;
            }
        }

        std::string output = line.str() + "\n";
        std::vector<char> data(output.begin(), output.end());

        return DSink->Write(data);
    }
};

CDSVWriter::CDSVWriter(std::shared_ptr<CDataSink> sink, char delimiter, bool quoteall)
    : DImplementation(std::make_unique<SImplementation>(sink, delimiter, quoteall)) {}

CDSVWriter::~CDSVWriter() = default;

bool CDSVWriter::WriteRow(const std::vector<std::string>& row) {
    return DImplementation->WriteRow(row);
}