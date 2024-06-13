/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package ftclib.archive;

import android.util.Xml;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Stack;

import trclib.archive.TrcSong;

/**
 * This class implements a parser of notated song in an xml file.
 */
public class FtcSongXml
{
    private final String instanceName;
    private final XmlPullParser parser;
    private final ArrayList<TrcSong> collection = new ArrayList<>();

    /**
     * Constructor: Create an instance of this object.
     *
     * @param instanceName specifies the instance name.
     * @param input specifies the input stream from which the notated song is read.
     */
    public FtcSongXml(String instanceName, InputStream input) throws XmlPullParserException, IOException
    {
        this.instanceName = instanceName;
        try
        {
            parser = Xml.newPullParser();
            parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false);
            parser.setInput(input, null);
            parser.nextTag();
            parseCollection();
        }
        finally
        {
            input.close();
        }
    }   //FtcSongXml

    /**
     * This method returns the song name.
     *
     * @return instance name.
     */
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the number of songs in the collection xml.
     *
     * @return number of songs in the collection.
     */
    public int getNumSongs()
    {
        return collection.size();
    }   //getNumSongs

    /**
     * This method returns the name of the song with the specified song index in the collection.
     *
     * @param index specifies the song index in the collection.
     * @return song name.
     */
    public String getSongName(int index)
    {
        return collection.get(index).toString();
    }   //getSongName

    /**
     * The method returns the song from the collection with the specified song index.
     *
     * @param index specifies the song index in the collection.
     * @return song in the collection with the specified index.
     */
    public TrcSong getSong(int index)
    {
        return collection.get(index);
    }   //getSong

    /**
     * The method returns the song from the collection with the specified song name.
     *
     * @param name specifies the song name to look for..
     * @return song in the collection with the specified name, null if none found.
     */
    public TrcSong getSong(String name)
    {
        TrcSong song = null;

        for (int i = 0; i < collection.size(); i++)
        {
            song = collection.get(i);
            if (name.equals(song.toString()))
            {
                break;
            }
            else
            {
                song = null;
            }
        }

        return song;
    }   //getSong

    /**
     * The method returns the song collection parsed from the XML file.
     *
     * @return song collection.
     */
    public TrcSong[] getCollection()
    {
        return collection.toArray(new TrcSong[0]);
    }   //getCollection

    /**
     * This method parses the song collection in the XML file. The collection starts with a collection tag and
     * contains multiple song tags.
     *
     * @throws XmlPullParserException if there is a parse error.
     * @throws IOException if there is file access error.
     */
    private void parseCollection() throws XmlPullParserException, IOException
    {
        parser.require(XmlPullParser.START_TAG, null, "collection");
        while (parser.next() != XmlPullParser.END_TAG)
        {
            //
            // Skip everything until we find the next start tag.
            //
            if (parser.getEventType() != XmlPullParser.START_TAG)
            {
                continue;
            }

            //
            // Check if the start tag is a song tag. If not, skip the entire tag including the nested tags in it.
            //
            String name = parser.getName();
            if (name.equals("song"))
            {
                collection.add(parseSong());
            }
            else
            {
                skipTag();
            }
        }
        parser.require(XmlPullParser.END_TAG, null, "collection");
    }   //parseCollection

    /**
     * This method parses the song in the XML file. The song starts with a song tag and contains one sequence tag
     * and multiple section tags.
     *
     * @throws XmlPullParserException if there is a parse error.
     * @throws IOException if there is file access error.
     */
    private TrcSong parseSong() throws XmlPullParserException, IOException
    {
        parser.require(XmlPullParser.START_TAG, null, "song");
        TrcSong song = new TrcSong(parser.getAttributeValue(null, "name"));
        while (parser.next() != XmlPullParser.END_TAG)
        {
            //
            // Skip everything until we find the next start tag.
            //
            if (parser.getEventType() != XmlPullParser.START_TAG)
            {
                continue;
            }

            //
            // Check if the start tag is a sequence tag or a section tag. If not, skip the entire tag
            // including the nested tags in it.
            //
            String name = parser.getName();
            if (name.equals("sequence"))
            {
                parseSequence(song);
            }
            else if (name.equals("section"))
            {
                parseSection(song);
            }
            else
            {
                skipTag();
            }
        }
        parser.require(XmlPullParser.END_TAG, null, "song");

        return song;
    }   //parseSong

    /**
     * This method parses the song sequence in the XML file. The song sequence starts with a sequence tag and
     * contains a sequence of section names separated by commas.
     *
     * @param song specifies the song object that the sequence belongs to.
     * @throws XmlPullParserException if there is a parse error.
     * @throws IOException if there is file access error.
     */
    private void parseSequence(TrcSong song) throws XmlPullParserException, IOException
    {
        parser.require(XmlPullParser.START_TAG, null, "sequence");
        StringBuilder sequence = new StringBuilder();
        while (parser.next() == XmlPullParser.TEXT)
        {
            sequence.append(parser.getText());
        }
        song.setSequence(sequence.toString());
        parser.require(XmlPullParser.END_TAG, null, "sequence");
    }   //parseSequence

    /**
     * This method parses the song section in the XML file. The song section starts with a section tag and
     * contains a sequence of notated notes separated by commas.
     *
     * @param song specifies the song object that the section belongs to.
     * @throws XmlPullParserException if there is a parse error.
     * @throws IOException if there is file access error.
     */
    private void parseSection(TrcSong song) throws XmlPullParserException, IOException
    {
        parser.require(XmlPullParser.START_TAG, null, "section");
        String name = parser.getAttributeValue(null, "name");
        StringBuilder notation = new StringBuilder();
        while (parser.next() == XmlPullParser.TEXT)
        {
            notation.append(parser.getText());
        }
        song.addSection(name, notation.toString());
        parser.require(XmlPullParser.END_TAG, null, "section");
    }   //parseSection

    /**
     * This method parses and skips  section in the XML file. The song section starts with a section tag and
     * contains a sequence of notated notes separated by commas.
     *
     * @throws XmlPullParserException if there is a parse error.
     * @throws IOException if there is file access error.
     */
    private void skipTag() throws XmlPullParserException, IOException
    {
        if (parser.getEventType() != XmlPullParser.START_TAG)
        {
            throw new XmlPullParserException("Expected start tag.");
        }

        Stack<String> tagNames = new Stack<>();
        tagNames.push(parser.getName());
        while (!tagNames.empty())
        {
            switch (parser.next())
            {
                case XmlPullParser.END_TAG:
                    String startTag = tagNames.pop();
                    String endTag = parser.getName();
                    if (!endTag.equals(startTag))
                    {
                        throw new XmlPullParserException(
                                "Unmatched end tag <" + endTag + "> (expected <" + startTag + ">)");
                    }
                    break;

                case XmlPullParser.START_TAG:
                    tagNames.push(parser.getName());
                    break;
            }
        }
    }   //skipTag

}   //class FtcSongXml
