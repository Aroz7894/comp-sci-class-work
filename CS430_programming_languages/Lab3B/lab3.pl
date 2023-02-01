#!/usr/local/bin/perl
#Andrew Rozniakowski
#2/2/15
#This program inputs a list of movie titles from a file and outputs #them in order of highest frequency with there counts. It has 3 
#subprograms, inputTable which imports the file of movie titles into 
#the program. It also has sortTable, which sorts the movie titles 
#based on highest frequency. And finally printTable which prints the #sorted table and frequency values.      

#main method
sub main{
    my %movie_titles = ();
    my $file = 'movies.dat';
    my %movie_titles = inputTable(\$file);
    my @sort_movies = sortTable(\%movie_titles);
    printTable(\@sort_movies , \%movie_titles);
    
}

#This sub takes a file as a parameter and opens the file and reads its data into a hash #table, counting the number of times each title is in the file   
sub inputTable {  
    my %titles = ();
    my $file_ref = @_;
    open(my $in, '<', $$file_ref) or die "Cant open movies.dat: $!";
    while (<$in>){
        if (exists($titles{$_})){
            $titles{$_} = $titles{$_} + 1;         
        }
        else{
            $titles{$_} = 1;
        }        
    }
    close $in or die "$in: $!";
    return %titles
}

#this sub sorts the hash table by highest frequency of the movie titles and puts them in an array
sub sortTable {
    my %movie_titles_ref = @_;
    my @keys = sort { $movie_titles_ref->{$b} <=> $movie_titles_ref->{$a} } keys(%$h);
    my @vals = @{$movie_titles_ref}{@keys};
    return @vals;
    
}

sub printTable{
    my ($sort_movies, $movie_titles_ref) = @_;
        print $$sort_movies + " " + $$movie_titles_ref{$$sort_movies};

&main();    #calls main method
