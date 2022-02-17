// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "TokenState.h"
#include "../Logging.h"

namespace Configuration {
    class Tokenizer {
        const char* start_;
        const char* current_;
        const char* end_;

        void skipToEol();

    protected:
        inline void Inc() {
            if (current_ != end_) {
                ++current_;
            }
        }
        inline char Current() const { return EndOfInput() ? '\0' : (*current_); }

        inline bool IsAlpha() {
            char c = Current();
            return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
        }

        inline bool IsSpace() { return Current() == ' '; }

        inline bool IsWhiteSpace() {
            if (EndOfInput()) {
                return false;
            }
            char c = Current();
            return c == ' ' || c == '\t' || c == '\f' || c == '\r';
        }

        inline bool IsIdentifierChar() { return IsAlpha() || IsDigit() || Current() == '_'; }

        inline bool IsEndLine() { return EndOfInput() || Current() == '\n'; }

        inline bool IsDigit() {
            char c = Current();
            return (c >= '0' && c <= '9');
        }

        inline char ToLower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

        inline bool EqualsCaseInsensitive(const char* input) {
            const char* tmp = current_;
            while (ToLower(*input) == ToLower(Current()) && *input != '\0') {
                Inc();
                ++input;
            }

            bool isSame = *input == '\0';  // Everything till the end of the input string is the same
            current_    = tmp;             // Restore situation
            return isSame;
        }

        inline bool EndOfInput() const { return current_ == end_; }

        // Results:
        struct TokenData {
            // The initial value for indent is -1, so when ParserHandler::enterSection()
            // is called to handle the top level of the YAML config file, tokens at
            // indent 0 will be processed.
            TokenData() :
                keyStart_(nullptr), keyEnd_(nullptr), indent_(-1), state(TokenState::Bof), sValueStart_(nullptr), sValueEnd_(nullptr) {}

            const char* keyStart_;
            const char* keyEnd_;
            int         indent_;

            TokenState state = TokenState::Bof;

            const char* sValueStart_;
            const char* sValueEnd_;
        } token_;

        void ParseError(const char* description) const;

    public:
        Tokenizer(const char* start, const char* end);
        void Tokenize();

        inline StringRange key() const { return StringRange(token_.keyStart_, token_.keyEnd_); }
        inline bool        eof() const { return token_.state == TokenState::Eof; }
        inline TokenState  state() const { return token_.state; }
        inline void        setState(TokenState state) { token_.state = state; }
        inline int         indent() const { return token_.indent_; }
    };
}
