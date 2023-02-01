--
-- Name: Andrew Rozniakowski
--
-- Write your queries below each comment. Please use good style (one clause
-- per line, JOIN syntax, indentation) and make sure all queries end with a
-- semicolon. When necessary, limit the output to the first 500 results.
--
-- DO NOT MODIFY OR DELETE ANY OTHER LINES!
--

--------------------------------------------------------------------------------
-- DBLP Queries
\c dblp
--------------------------------------------------------------------------------

--
\echo QUERY #1
\echo
-- List the coauthors of Jennifer Widom who have written more publications
-- than she has.
--
-- You must write the query in a generic way that would work for any author.
-- Hint: The only table you need is auth. Use subqueries to do the counting.
--
-- Schema: coauthor
--  Order: coauthor

WITH coauth AS(
  SELECT DISTINCT b.author
  FROM auth AS a
    JOIN auth AS b ON a.dblp_key = b.dblp_key
      AND a.author <> b.author
  WHERE a.author = 'Jennifer Widom')
SELECT author AS coauthor
FROM coauth
  NATURAL JOIN auth
GROUP BY author
HAVING COUNT(author) > (SELECT COUNT(a.dblp_key)
FROM auth AS a
WHERE a.author = 'Jennifer Widom')
ORDER BY author;

--
\echo QUERY #2
\echo
-- Find all publications since Jan 1, 2000 that have the words 'K-12 education'
-- anywhere in the title (i.e., the user typed that into a simple search box).
--
-- Schema: year, title, dblp_type, booktitle, journal, volume, issnum, pages
--  Order: dblp_mdate, dblp_key

SELECT year, title, dblp_type, booktitle, journal, volume, issnum, pages 
FROM publ
WHERE title @@ '%K-12%education%'
  AND year >= 2000
ORDER BY dblp_mdate, dblp_key;

--
\echo QUERY #3
\echo
-- Search for publications that have a title similar to our textbook's title.
-- Set the similarity threshold to 0.45 before running your query.
--
-- Schema: dblp_key, year, title, similarity
--  Order: similarity DESC, dblp_key

SELECT SET_LIMIT(0.45);
SELECT dblp_key, year, title, similarity(title, 'A First Course in Database Systems') AS similarity
FROM publ
  WHERE title % 'A First Course in Database Systems'
ORDER BY similarity DESC, dblp_key;


--------------------------------------------------------------------------------
-- TPC-H Queries
\c tpch
--------------------------------------------------------------------------------

--
\echo QUERY #4
\echo
-- Which parts are supplied in the Europe region?
-- Display in order of retail price and part number.
--
-- Schema: p_partkey, p_name, p_retailprice

SELECT DISTINCT p_partkey, p_name, p_retailprice
FROM PART
  JOIN PARTSUPP ON p_partkey = ps_partkey
  JOIN SUPPLIER ON ps_suppkey = s_suppkey
  JOIN NATION ON s_nationkey = n_nationkey
  JOIN REGION ON n_regionkey = r_regionkey
WHERE r_name = 'EUROPE'
ORDER BY p_retailprice, p_partkey
LIMIT 500;

--
\echo QUERY #5
\echo
-- Find the minimum cost supplier for each part.
-- Display in ascending order by part number.
--
-- You must use a WITH clause to receive full credit.
-- Hint: Find the minimum cost of each part first.
--
-- Schema: ps_partkey, ps_suppkey, min_supplycost

WITH supply AS (
  SELECT ps_partkey, MIN(ps_supplycost) AS min_supplycost
  FROM partsupp
  GROUP BY ps_partkey)
SELECT ps.ps_partkey, ps.ps_suppkey, s.min_supplycost
FROM supply AS s
  JOIN partsupp AS ps ON s.ps_partkey = ps.ps_partkey
WHERE ps_supplycost = s.min_supplycost
ORDER BY ps.ps_partkey
LIMIT 500;


--
\echo QUERY #6
\echo
-- Which urgent priority orders have only one line item?
-- Display in ascending order by the order number.
--
-- Schema: o_orderkey, o_custkey, o_orderstatus

SELECT o_orderkey, o_custkey, o_orderstatus
FROM ORDERs
  JOIN lineitem AS l ON o_orderkey = l_orderkey
WHERE o_orderpriority = '1-URGENT'
GROUP BY o_orderkey
HAVING COUNT(o_orderkey) = 1
ORDER BY o_orderkey ASC
limit 500;
