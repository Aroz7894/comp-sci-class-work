--
-- Name: Andrew Rozniakowski
--
-- Please write your query below each comment.
-- Make sure all queries end with a semicolon.
--
-- DO NOT MODIFY OR DELETE ANY OTHER LINES!
--

--------------------------------------------------------------------------------
-- IMDb Queries
\c imdb
--------------------------------------------------------------------------------

--
\echo QUERY #1
\echo
-- For the movies named Tangled, what kind of movie was it and what year was
-- it released?
--
-- Schema: kind varchar(15), year integer
--  Order: year, kind

SELECT k.kind, m.year
FROM movie AS m, kind_type as k
WHERE title = 'Tangled'
ORDER BY year, kind;

--
\echo QUERY #2
\echo
-- Show all actors and actresses for the movie Inception (2010).
--
-- Schema: real_name text, char_name text
--  Order: nr_order

SELECT p.name, ch.name
FROM movie AS m
  JOIN cast_info AS ci ON m.id = ci.movie_id
  JOIN person AS p ON ci.person_id = p.id
  JOIN character AS ch ON ci.char_id = ch.id
WHERE m.title = 'Inception' 
  AND m.year = 2010 
  AND m.kind_id = 1
  AND (ci.role_id = 1 OR  ci.role_id = 2)
ORDER BY ci.nr_order;

--
\echo QUERY #3
\echo
-- For each tv movie in 2014 that has runtime information,
-- what is the plot of the movie? If there is no plot record
-- in the database, simply use NULL for that movie's plot.
-- Only display the first 20 results (there are 1112 total).
--
-- Schema: title text, runtime text, plot text
--  Order: runtime

SELECT m.title, mi2.info, mi1.info
FROM movie as m
JOIN movie_info AS mi1 ON mi1.movie_id = m.id
  AND mi1.info_id = 1
LEFT JOIN movie_info as mi2 ON mi2.movie_id = mi1.movie_id
  AND mi2.info_id = 98 
WHERE m.kind_id = 3 
  AND year = 2014
ORDER BY mi1.info
LIMIT 20;

--
\echo QUERY #4
\echo
-- List the top 10 actors and actresses with "Smith" in their name.
-- By "top" we mean those who have been in the most movies of any
-- type. If a person plays more than one role in a particular movie,
-- then that movie should only be counted once for that person.
--
-- Schema: name text, count bigint
--  Order: descending count

SELECT p.name, COUNT(DISTINCT m.id)
FROM movie AS m
  JOIN cast_info AS ci ON m.id = ci.movie_id
  JOIN person AS p ON ci.person_id = p.id 
WHERE p.name LIKE '%Smith%' 
  AND (ci.role_id = 1 
    OR ci.role_id = 2)
GROUP BY p.id
ORDER BY COUNT(DISTINCT m.id) DESC
LIMIT 10;

--------------------------------------------------------------------------------
-- VLDS Queries
\c vlds
--------------------------------------------------------------------------------

--
\echo QUERY #5
\echo
-- How many students were enrolled in each grade at Harrisonburg High in 2011?
--
-- Schema: grade_code text, fall_membership_cnt integer
--  Order: grade_code

SELECT grade_code, MAX(fall_membership_cnt)
FROM fall_membership
WHERE school_year = '2011-2012'
  AND sch_name = 'Harrisonburg High'
  AND grade_code IS NOT NULL 
GROUP BY grade_code
ORDER BY grade_code;

--
\echo QUERY #6
\echo
-- What are the five largest schools in terms of their 2011 enrollment?
--
-- Schema: div_name text, sch_name text, fall_membership_cnt integer
--  Order: descending fall_membership_cnt

SELECT div_name, sch_name, MAX(fall_membership_cnt)
FROM fall_membership
WHERE school_year = '2011-2012'
  AND federal_race_code IS NULL
  AND gender IS NULL
  AND disability_flag IS NULL
  AND lep_flag IS NULL
  AND disadvantaged_flag IS NULL
  AND grade_code IS NULL
  AND sch_name IS NOT NULL
GROUP BY sch_name, div_name
ORDER BY MAX(fall_membership_cnt) DESC
LIMIT 5;


--
\echo QUERY #7
\echo
-- List the SOL results for 8th graders from Skyline Middle in 2011-2012,
-- showing all scores for male and female students as well as both genders.
--
-- Schema: test text, gender text, avg_sol_scale_score integer,
--         pass_advanced_rate real, pass_prof_rate real,
--         pass_rate real, fail_rate real
--  Order: test, gender

SELECT test, gender, avg_sol_scale_score, pass_advanced_rate, pass_prof_rate,
       pass_rate, fail_rate
FROM sol_test_data
WHERE school_year = '2011-2012'
  AND sch_name = 'Skyline Middle'
  AND sch_num = '0111'
  AND div_num = '113'
  AND test_level = '8'
  AND federal_race_code IS NULL 
  AND lep_flag IS NULL 
  AND disability_flag IS NULL 
  AND disadvantaged_flag IS NULL
  AND (test = 'English Reading'
    OR test = 'Mathematics'
    OR test = 'Science'
    OR test = 'Writing')
ORDER BY test, gender;


\echo QUERY #8
\echo
-- In 2011-2012, which schools had more than 1000 students enrolled
-- and achieved an SOL pass rate of over 90% in 8th grade math?
--
-- Schema: div_num text, sch_name text
--  Order: sch_name, div_num

SELECT fm.div_name,  fm.sch_name
FROM fall_membership AS fm
  JOIN sol_test_data AS s ON s.sch_num = fm.sch_num 
WHERE fm.fall_membership_cnt > 1000
  AND fm.div_num = s.div_num
  AND fm.school_year = '2011-2012'
  AND fm.federal_race_code IS NULL
  AND fm.gender IS NULL
  AND fm.disability_flag IS NULL
  AND fm.lep_flag IS NULL
  AND fm.disadvantaged_flag IS NULL
  AND s.pass_rate > '90'
  AND s.test_level = '8'
  AND s.subject = 'MATH'
  AND s.school_year = '2011-2012'
  AND s.federal_race_code IS NULL
  AND s.gender IS NULL
  AND s.disability_flag IS NULL
  AND s.lep_flag IS NULL
  AND s.disadvantaged_flag IS NULL 
ORDER BY  fm.sch_name, fm.div_name;

