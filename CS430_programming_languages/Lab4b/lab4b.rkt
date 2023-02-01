#lang racket

;
; Start the program.
; No parameters, no return value.
;
(define (start)
  (write-string "Enter a sentence: ")
  (let ( (sentence (read-sentence-as-list)))
    (cond
      ( (assign sentence) (write-string "Accepted") (newline) )
      ( else (write-string "Not accepted") (newline) )
    )
  )
)

;
; Return true if the provided list matches the <assign> rule, false if not.
; params: list - a list of symbols constituting the expression to be tested
; return: true if the list is a valid sentence, false otherwise
;
(define (assign list)
  (cond
    ( (not (list? list)) '() )  ; precondition test
    
    ( (and 
        (var (car list)) 
        (eq? '= (car (cdr list)))
        (expr (cdr (cdr list)))
       )
       #true 
    )
    ( else #false )
  )
)

;This function takes in a list of expressions and returns true if the list matches
;the <expr> rule
;params: list - a list of symbols constitiuting the expression to be tested 
;return: true if the list is a valid sentence, false otherwise 
(define (expr list)
   (cond
    ( (not (list? list)) '() )  ; precondition test
    
    ( (and 
        (term (car list))
        (eq? '() (cdr list))
    ( (or
        (and
        (term (car list)) 
        (op (car (cdr list)))
        (term (cdr (cdr list)))
        (eq? '() (cdr (cdr (cdr list))))
       )
       )
         )
       #true 
    )
    ( else #false )
  )
)


;This function takes in a symbol and returns true if the symbol is a symbol valid of the 
;<term> rule
;params: a single symbol - a symbol to be tested whether its contained in the <term> rule 
;return: true if the symbol is a valid symbol, false otherwise
(define (term symbol)
  (cond
    ( (not (symbol? symbol)) null )  ; precondition test
    
    ( (and 
        (or
            (int symbol)         
            (var symbol)
        )
      ) 
       #true 
    )
    ( else #false )
  )
)

;This function takes in a symbol and returns true if the symbol is a valid symbol of the 
;<op> rule
;params: a single symbol - a symbol to be tested whether its contained in the <op> rule 
;return: true if the symbol is a valid symbol, false otherwise
(define (op symbol)
(cond
    ( (not (symbol? symbol)) null )  ; precondition test
    
    ( (and
        (member symbol '(\+ \- \* \/ \^ \%) )
       )
       #true 
    )
    ( else #false )
  )
)
;This function takes in a symbol and returns true if the symbol is a valid symbol of the 
;<var> rule
;params: a single symbol - a symbol to be tested whether its contained in the <var> rule 
;return: true if the symbol is a valid symbol, false otherwise
(define (var symbol)
  (cond
    ( (not (symbol? symbol)) null )  ; precondition test
    
    ( (and
        (member symbol '(\a \b \c \d \e \f \g \h \i \j \k \l \m \n \o \p \q \r \s \t \u \v \w \x \y \z) )
       )
       #true 
    )
    ( else #false )
  )
)

;This function takes in a symbol and returns true if the symbol is a valid symbol of the 
;<int> rule
;params: a single symbol - a symbol to be tested whether its contained in the <int> rule 
;return: true if the symbol is a valid symbol, false otherwise
(define (int symbol)
  (cond
    ( (not (symbol? symbol)) null )  ; precondition test
    
    ( (and
        (member symbol '(\0 \1 \2 \3 \4 \5 \6 \7 \8 \9) )
       )
       #true 
    )
    ( else #false )
  )
)
 
;
; Read a sentence from input, and return a list of the symbols that it contains.
; Each character except blank is considered to be a separate symbol.
; params: none
; return: none
;
(define (read-sentence-as-list)
  (build-symbol-list (string->list (read-line)))
)

;
; Converts a list of characters into a list of equivalent symbols.
; Any blanks in the list are bypassed.
; params: a list of characters
; return: a list of symbols, equivalent to the characters
;
(define (build-symbol-list char-list)
  (cond
    ( (not (list? char-list) ) '() )  ; precondition test
    
    ( (empty? char-list) '() )
    ( (eq? #\space (car char-list)) (build-symbol-list (cdr char-list)) )
    ( else (cons (string->symbol (string (car char-list))) (build-symbol-list (cdr char-list)) ) )
  )
)

