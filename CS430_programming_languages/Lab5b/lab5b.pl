% Lab5
%Andrew Rozniakowski
%2/18/15

% Grammar:
% <assign> => <id> = <expr>
% <expr>   => { <term> <op> <expr>  | ( <expr> )  |  <term> }sel  
% <op>     => { * | + | / | - }sel 
% <term>   => { <id> | <digit> }sel 
% <id>     => { a | b | c }sel 
% <digit>  => { 0 | 1 | ... | 9 }sel 
%  Note: Input digits will be in character form, '0', '1', etc.
   
%
% Start the program.
% In gprolog, enter "start." to activate this rule.
%
start :-
  read_sentence(Sentence),
  assign(Sentence).
%
% Input a sentence, then decompose it into a list of symbols. 
% User input should be quoted (single quotes), e.g., Enter a sentence: 'a = b * c'
%
% param (out): SentenceList  The list of symbols input by the user is bound to this variable
%
read_sentence(SentenceList) :-
   print('Enter a sentence: '),
   read_token(Input),
   write_term_to_chars(InputList, Input, []),
   delete(InputList, ' ', SentenceList).

%
% Determine if the Sentence is valid according to the grammar
%
% param (in): Sentence - A list of symbols that make up the sentence to be evaluated
%
assign(Sentence) :-
  list(Sentence), length(Sentence, Length), >=(Length, 3),    %precondition test
  =(Sentence, [First, Second | Tail] ),                       %split the sentence
  id(First),
  =(Second, '='),
  expr(Tail).
% DO NOT MAKE CHANGES ABOVE THIS POINT! /

% skeleton rules, to be completed
%Determining if the expr rule is valid
expr(E):- 
	list(E), length(E, Len), >=(Len, 3),        %precondition test
	(E, [H1, H2 | Tail]), 
	term(H1), op(H2), expr(Tail) ;
 	list(E), length(E, Len), >=(Len, 3),        %precondition test
	(E, [Head | Tail]), =(Head ,'('), 
	reverse(Tail, Rev_E),
    (Rev_E, [Rev_Head|Rev_Tail], 
    =(Rev_Head ,')'), reverse(Rev_Tail, Reg_Tail),
    expr(Reg_Tail);
    list(E), length(E, Len), =(Len, 1),         %precondition test
	(E, [Head | Tail]), 
	term(Head).

%Determining if the op rule is valid
op(Op) :-
member(Op, ['*', '+', '/', '-']).

%Determining if the term rule is valid
term(T) :- digit(T) ; id(T).

%Determining if the id rule is valid
id(I) :-
	member(Op, ['a', 'b', 'c']).

%Determining if the digit rule is valid
digit(D) :- 
	atom_length(D, 1), D @>= '0' , D @=< '9'.

